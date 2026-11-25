// uvpack_cli: pack polygons described in JSON, emit JSON transforms.
//
// Input:  {"target":[minx,miny,maxx,maxy], "islands":[{"id":0,"outer":[[x,y],...],"holes":[[[x,y],...]]}],
//          "options":{"margin":0.003,"stripe_height":0.002,"allow_rotation":true,"rotation_step":90,
//                     "prealign":true,"trials":4,"seed":1,
//                     "scale_mode":"max"|"fixed","scale":1.0,"bracket_rounds":14,"time_budget":0}}
// Output: {"dialect":"uvpack","valid":bool,"scale":s,"islands":[{"id":..,"angle":..,"transform":[9]}],"stats":{...}}
//
// With "stream": true the search's realtime solutions go to stdout as they happen,
// one JSON object per line: {"event":"improved","dialect":"uvpack","scale":..,"elapsed":..,
// "islands":[..]} for every committed improvement, then exactly one {"event":"done",
// <the full result above>} line. SIGINT stops the search; the done line still comes
// out, with "cancelled": true.
//
// Subcommands: `uvpack_cli --version` reports the vocabulary, the version and whether
// the GPU worker is compiled in; `uvpack_cli devices` reports what that worker can see.
//
// The request parsing lives in cli/request.cpp and the reply in cli/response.cpp.
#include "cli.hpp"
#include "cancellation.hpp"
#include "uvpack/compaction.hpp"
#include "uvpack/geometry.hpp"
#include "uvpack/gpu.hpp"
#include <chrono>
#include <csignal>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#include <shellapi.h>
#include <fcntl.h>
#include <io.h>
#endif

using namespace uvpack;

static void on_interrupt(int) { cli::interrupted.store(true,std::memory_order_relaxed); }

// `uvpack_cli --version`: the vocabulary this executable speaks, before it is asked
// anything, so a client can check it without building a request first.
static int print_version() {
    auto out = json::parse(metadata::json).as_object();
    out["gpu"] = gpu::compiled_in();
    std::cout << json::dump(out) << '\n';
    return 0;
}

// `uvpack_cli devices`: what the GPU worker can see, and why it cannot when it cannot.
static int print_devices() {
    json::Object out;
    out["compiled_in"] = gpu::compiled_in();
    out["available"] = gpu::available();
    out["diagnostic"] = gpu::diagnostic();
    json::Array list;
    for (const auto& device : gpu::devices()) {
        json::Object item;
        item["name"] = device.name;
        item["type"] = device.type;
        item["vendor_id"] = double(device.vendor_id);
        item["device_id"] = double(device.device_id);
        item["device_local_mib"] = double(device.device_local_bytes >> 20);
        item["api"] = std::to_string(device.api_major) + "." + std::to_string(device.api_minor) +
                      "." + std::to_string(device.api_patch);
        item["max_workgroup_invocations"] = double(device.max_workgroup_invocations);
        item["max_shared_memory_bytes"] = double(device.max_shared_memory_bytes);
        item["portability"] = device.portability;
        list.push_back(item);
    }
    out["devices"] = list;
    if (gpu::available()) {
        const auto failure = gpu::self_test();
        out["self_test"] = failure.empty() ? std::string("ok") : failure;
    }
    std::cout << json::dump(out) << '\n';
    return gpu::available() ? 0 : 2;
}

static int run_cli(int argc, char** argv) {
    const std::string first = argc > 1 ? std::string(argv[1]) : std::string();
    if (first == "devices") return print_devices();
    if (first == "--version" || first == "version") return print_version();
    bool stream_response=false;
    try {
        std::optional<std::filesystem::path> cancel_file;
        std::string input_file;
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--cancel-file") {
                if(cancel_file || ++i==argc || !*argv[i])throw std::runtime_error("--cancel-file needs one nonempty path");
                cancel_file=std::filesystem::u8path(argv[i]);
            } else {
                if(!input_file.empty() || (arg!="-" && arg.rfind("--",0)==0))throw std::runtime_error("unexpected CLI argument: "+arg);
                input_file=arg;
            }
        }
        std::string text;
        if (!input_file.empty() && input_file != "-") {
            std::ifstream in(std::filesystem::u8path(input_file),std::ios::binary);
            if (!in) throw std::runtime_error(std::string("cannot open ") + input_file);
            text=json::read_text(in);
        } else {
            text=json::read_text(std::cin);
        }
        const auto doc=json::parse(text);
        const auto* op=doc.find("operation");
        const auto operation=op?op->as_string():"pack";
        if(const auto* options=doc.find("options"))stream_response=options->bool_or("stream",false);
        std::signal(SIGINT, on_interrupt);
        std::signal(SIGTERM, on_interrupt);
        if(operation=="validate") {
            CancellationToken token;
            cli::RequestCancellation cancellation(token,cancel_file);
            auto result=cli::validate_request(doc,token);
            cancellation.finish();
            // A certificate completed before a late signal remains valid, just
            // as a fully validated packing incumbent can survive cancellation.
            result["cancelled"]=token.requested();
            if(stream_response)result["event"]=std::string("done");
            std::cout<<json::dump(result)<<std::endl;
            return result.at("valid").as_bool()?0:2;
        }
        if(operation=="normalize") {
            CancellationToken token;
            cli::RequestCancellation cancellation(token,cancel_file);
            auto result=cli::normalize_request(doc,token);
            cancellation.finish();
            result["cancelled"]=token.requested();
            if(stream_response)result["event"]=std::string("done");
            std::cout<<json::dump(result)<<std::endl;
            return result.at("accepted").as_bool()?0:2;
        }
        if(operation!="pack") throw std::runtime_error("operation must be pack, validate or normalize");
        auto request = cli::parse_request(doc);
        auto& options = request.options;
        const auto& target = request.target;
        const auto& islands = request.islands;
        const auto& obstacles = request.obstacles;

        SearchStats stats;
        cli::RequestCancellation cancellation(options.cancel,cancel_file);
        const auto t0 = std::chrono::steady_clock::now();
        auto elapsed_since_start = [&t0] {
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        };
        if (request.stream) {
            // Realtime solutions. Written from the search thread, flushed immediately so
            // an interactive client (the Blender modal operator) can draw each one.
            options.on_improved = [&](const Layout& l) {
                auto event=cli::stream_event(request,l,elapsed_since_start());
                if(!event.empty())std::cout<<json::dump(event)<<std::endl;
            };
        }
        double achieved_margin = options.margin;
        Layout layout;
        if (request.require_complete && !request.input_report.at("complete").as_bool()) {
            layout.valid=false; // No search or misleading incumbent after input loss.
        } else if (request.scale_mode == "open_rescale") {
            // Diagnostic: one open-canvas pack at the area bound, then rescale the whole
            // layout to the target (gaps shrink with it). Mirrors a one-pass engine.
            double area = 0;
            for (const auto& i : islands) area += measure_island(i).area * i.pre_scale * i.pre_scale * options.target_aspect;
            const double upper = std::sqrt((target.max_x - target.min_x) * options.target_aspect * (target.max_y - target.min_y) / area);
            const double s0 = upper * options.initial_fraction;
            PackOptions open = options; open.trials = 8;
            layout = pack_open(islands, target, s0, open, 3.0, &stats);
            if (layout.valid) {
                const double extent = std::max(layout.occupied_width / ((target.max_x - target.min_x) * options.target_aspect), layout.occupied_height / (target.max_y - target.min_y));
                for (auto& p : layout.placements) { p.island_scale /= extent; p.offset.x /= extent; p.offset.y /= extent; p.source_origin.x /= extent; p.source_origin.y /= extent; }
                layout.scale = s0 / extent;
            }
            // This diagnostic mode drives pack_open itself, so it has to mirror itself too.
            apply_origin(layout, options, target);
        } else if (request.scale_mode == "fixed") {
            layout = pack_fixed_scale(islands, target, request.fixed_scale, options, &stats, obstacles);
            // Standalone use: at a fixed scale there is no scale to gain, so the
            // compaction runs its zero-growth polish (slack + continuous rotation).
            if (options.compaction_time > 0 && layout.valid && !options.cancel.requested()) {
                CompactionStats compaction;
                PackOptions polish_only = options;
                polish_only.compaction_delta = 0; // The scale is fixed by definition here.
                layout = compact_layout(islands, target, layout, polish_only, obstacles, &compaction);
                stats.compaction_attempts = compaction.attempts;
                stats.compaction_accepted = compaction.accepted;
                stats.compaction_seconds = compaction.seconds;
                stats.compaction_rotations = compaction.rotations_applied;
                stats.compaction_threads = compaction.threads;
                stats.compaction_rounds = compaction.rounds;
                stats.compaction_start_scale = compaction.start_scale;
                stats.compaction_skipped = compaction.skipped;
            }
        }
        else if (request.scale_mode == "max_margin") layout = pack_max_margin(islands, target, request.fixed_scale, options, achieved_margin, &stats, obstacles);
        else layout = pack_max_scale(islands, target, options, &stats, obstacles);
        stats.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

        cancellation.finish();
        auto out = cli::result_json(request, layout, stats, achieved_margin);
        if (request.stream) out["event"] = std::string("done");
        std::cout << json::dump(out) << std::endl;
        return out["valid"].as_bool() ? 0 : 2;
    } catch (const std::exception& ex) {
        std::cout << "{\"dialect\":\"" << cli::dialect << "\",\"valid\":false,\"error\":"
                  << json::dump(json::Value(json::diagnostic(ex.what())))
                  << (stream_response?",\"event\":\"done\"":"") << "}\n";
        return 1;
    }
}

#ifdef _WIN32
int main() {
    // Read JSON bytes exactly: CRT text mode treats CTRL-Z as EOF and could
    // accept a valid prefix while silently ignoring trailing malformed data.
    if(_setmode(_fileno(stdin),_O_BINARY)==-1) {
        std::cout<<"{\"dialect\":\"uvpack\",\"valid\":false,\"error\":\"cannot set binary stdin\"}\n";
        return 1;
    }
    // CRT narrow argv loses characters outside the active Windows code page.
    int argc=0;
    auto wide=CommandLineToArgvW(GetCommandLineW(),&argc);
    if(!wide){std::cout<<"{\"dialect\":\"uvpack\",\"valid\":false,\"error\":\"cannot read Windows command line\"}\n";return 1;}
    struct FreeArguments {LPWSTR* value;~FreeArguments(){LocalFree(value);}} free_arguments{wide};
    std::vector<std::string> arguments;arguments.reserve(static_cast<std::size_t>(argc));
    for(int i=0;i<argc;++i) {
        const int bytes=WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,wide[i],-1,nullptr,0,nullptr,nullptr);
        if(!bytes){std::cout<<"{\"dialect\":\"uvpack\",\"valid\":false,\"error\":\"invalid command-line Unicode\"}\n";return 1;}
        std::string text(static_cast<std::size_t>(bytes),'\0');
        WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,wide[i],-1,text.data(),bytes,nullptr,nullptr);
        text.pop_back();arguments.push_back(std::move(text));
    }
    std::vector<char*> argv;for(auto& argument:arguments)argv.push_back(argument.data());
    return run_cli(argc,argv.data());
}
#else
int main(int argc,char** argv){return run_cli(argc,argv);}
#endif
