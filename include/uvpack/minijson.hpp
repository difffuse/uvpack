#pragma once
// Bounded JSON for the CLI. Numbers are finite doubles; strings are valid UTF-8.
// Defaults apply only to absent object fields. Present fields retain their types.
#include <cmath>
#include <charconv>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <locale>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <locale.h>
#ifdef __APPLE__
#include <xlocale.h>
#endif

namespace uvpack::json {
struct Limits {
    std::size_t max_input_bytes = 64 * 1024 * 1024;
    std::size_t max_depth = 128;
    std::size_t max_values = 4 * 1024 * 1024;
};

struct Value;
using Array = std::vector<Value>;
using Object = std::map<std::string, Value>;
struct Value {
    enum Kind { Null, Bool, Number, String, ArrayKind, ObjectKind } kind = Null;
    bool b = false;
    // A parsed decimal can have a nonzero fraction even when double rounding
    // erases it. Integer schema fields must not accept that rounded identity.
    bool fractional_lexeme = false;
    double num = 0;
    std::string str;
    std::shared_ptr<Array> arr;
    std::shared_ptr<Object> obj;

    Value() = default;
    Value(bool v) : kind(Bool), b(v) {}
    Value(double v) : kind(Number), num(v) {}
    Value(int v) : kind(Number), num(v) {}
    Value(std::size_t v) : kind(Number), num(double(v)) {}
    Value(const char* v) : kind(String), str(v) {}
    Value(std::string v) : kind(String), str(std::move(v)) {}
    Value(Array v) : kind(ArrayKind), arr(std::make_shared<Array>(std::move(v))) {}
    Value(Object v) : kind(ObjectKind), obj(std::make_shared<Object>(std::move(v))) {}

    bool is_null() const { return kind == Null; }
    bool is_array() const { return kind == ArrayKind; }
    bool is_object() const { return kind == ObjectKind; }
    const Array& as_array() const { if (kind != ArrayKind) throw std::runtime_error("json: expected array"); return *arr; }
    const Object& as_object() const { if (kind != ObjectKind) throw std::runtime_error("json: expected object"); return *obj; }
    double as_number() const { if (kind != Number) throw std::runtime_error("json: expected number"); return num; }
    bool as_bool() const { if (kind != Bool) throw std::runtime_error("json: expected bool"); return b; }
    const std::string& as_string() const { if (kind != String) throw std::runtime_error("json: expected string"); return str; }
    const Value* find(const std::string& key) const {
        if (kind != ObjectKind) return nullptr;
        auto it = obj->find(key);
        return it == obj->end() ? nullptr : &it->second;
    }
    const Value* optional(const std::string& key, Kind expected, const char* type) const {
        as_object();
        const auto* v = find(key);
        if (v && v->kind != expected) throw std::runtime_error("json: field '" + key + "' must be " + type);
        return v;
    }
    double number_or(const std::string& key, double fallback) const {
        auto v = optional(key, Number, "number"); return v ? v->num : fallback;
    }
    bool bool_or(const std::string& key, bool fallback) const {
        auto v = optional(key, Bool, "bool"); return v ? v->b : fallback;
    }
    std::string string_or(const std::string& key, const std::string& fallback) const {
        auto v = optional(key, String, "string"); return v ? v->str : fallback;
    }
};

namespace detail {
// A private, immutable C numeric locale preserves strtod's correctly rounded
// subnormals without changing the host's global/thread locale. Floating-point
// from_chars is unavailable on older supported Apple deployment targets, and
// iostream extraction treats representable subnormals as a range error there.
inline double decimal(const char* begin, char** end) {
    struct NumericLocale {
#ifdef _WIN32
        _locale_t value = _create_locale(LC_NUMERIC, "C");
        ~NumericLocale() { if(value) _free_locale(value); }
#else
        locale_t value = newlocale(LC_NUMERIC_MASK, "C", nullptr);
        ~NumericLocale() { if(value) freelocale(value); }
#endif
        NumericLocale() { if(!value) throw std::runtime_error("json: cannot create numeric locale"); }
    };
    static const NumericLocale locale;
#ifdef _WIN32
    return _strtod_l(begin,end,locale.value);
#else
    return strtod_l(begin,end,locale.value);
#endif
}
// A zero length denotes an invalid encoding. Reject overlong sequences,
// surrogates, out-of-range scalars and incomplete/stray continuation bytes.
inline std::size_t utf8_length(const std::string& s, std::size_t at) {
    if (at >= s.size()) return 0;
    const auto lead = static_cast<unsigned char>(s[at]);
    if (lead < 0x80) return 1;
    unsigned code = 0, minimum = 0;
    std::size_t count = 0;
    if (lead >= 0xc2 && lead <= 0xdf) { count=2; minimum=0x80; code=lead&0x1f; }
    else if (lead >= 0xe0 && lead <= 0xef) { count=3; minimum=0x800; code=lead&0x0f; }
    else if (lead >= 0xf0 && lead <= 0xf4) { count=4; minimum=0x10000; code=lead&7; }
    else return 0;
    if (count > s.size()-at) return 0;
    for (std::size_t j=1; j<count; ++j) {
        const auto c=static_cast<unsigned char>(s[at+j]);
        if ((c&0xc0)!=0x80) return 0;
        code=(code<<6)|(c&0x3f);
    }
    return code<minimum || code>0x10ffff || (code>=0xd800 && code<=0xdfff) ? 0 : count;
}
inline void append_utf8(std::string& out, unsigned code) {
    if (code<0x80) out+=static_cast<char>(code);
    else if (code<0x800) {
        out+=static_cast<char>(0xc0|(code>>6)); out+=static_cast<char>(0x80|(code&0x3f));
    } else if (code<0x10000) {
        out+=static_cast<char>(0xe0|(code>>12)); out+=static_cast<char>(0x80|((code>>6)&0x3f));
        out+=static_cast<char>(0x80|(code&0x3f));
    } else {
        out+=static_cast<char>(0xf0|(code>>18)); out+=static_cast<char>(0x80|((code>>12)&0x3f));
        out+=static_cast<char>(0x80|((code>>6)&0x3f)); out+=static_cast<char>(0x80|(code&0x3f));
    }
}
inline bool digit(char c) { return c>='0' && c<='9'; }
} // namespace detail

class Parser {
public:
    explicit Parser(const std::string& text, Limits limits={}) : s_(text), limits_(limits) {}
    Value parse() {
        if (s_.size()>limits_.max_input_bytes) fail("input byte limit exceeded");
        auto v=value(0); ws(); if (i_!=s_.size()) fail("trailing characters"); return v;
    }
private:
    const std::string& s_;
    Limits limits_;
    std::size_t i_=0, values_=0;
    [[noreturn]] void fail(const char* what) {
        throw std::runtime_error(std::string("json: ")+what+" at offset "+std::to_string(i_));
    }
    void ws() {
        while (i_<s_.size() && (s_[i_]==' ' || s_[i_]=='\t' || s_[i_]=='\r' || s_[i_]=='\n')) ++i_;
    }
    bool eat(char c) { ws(); if (i_<s_.size() && s_[i_]==c) { ++i_; return true; } return false; }
    Value value(std::size_t depth) {
        if (depth>limits_.max_depth) fail("nesting limit exceeded");
        if (values_>=limits_.max_values) fail("value count limit exceeded");
        ++values_;
        ws(); if (i_>=s_.size()) fail("unexpected end");
        const char c=s_[i_];
        if (c=='{') return object(depth);
        if (c=='[') return array(depth);
        if (c=='"') return Value(string());
        if (c=='t') { literal("true"); return Value(true); }
        if (c=='f') { literal("false"); return Value(false); }
        if (c=='n') { literal("null"); return Value(); }
        return number();
    }
    void literal(const char* word) {
        const std::string w(word);
        if (s_.compare(i_,w.size(),w)!=0) fail("bad literal");
        i_+=w.size();
    }
    bool fractional(std::size_t begin,std::size_t mantissa_end) const {
        std::int64_t fractional_digits=0,trailing_zeros=0;
        bool after_point=false,nonzero=false;
        for(auto at=begin;at<mantissa_end;++at) {
            const char c=s_[at];
            if(c=='.'){after_point=true;continue;}
            if(!detail::digit(c))continue;
            if(after_point)++fractional_digits;
            if(c=='0')++trailing_zeros;
            else {nonzero=true;trailing_zeros=0;}
        }
        if(!nonzero)return false;
        std::int64_t exponent=0;
        if(mantissa_end<i_) {
            auto at=mantissa_end+1;
            const bool negative=s_[at]=='-';
            if(s_[at]=='+' || s_[at]=='-')++at;
            // Saturating at the token length suffices for this comparison and
            // avoids overflow on arbitrarily long exponent spellings.
            const auto bound=static_cast<std::int64_t>(i_-begin);
            for(;at<i_;++at) {
                const auto digit=std::int64_t(s_[at]-'0');
                if(exponent>bound/10 || exponent*10+digit>bound)exponent=bound;
                else exponent=exponent*10+digit;
            }
            if(negative)exponent=-exponent;
        }
        return fractional_digits-trailing_zeros>exponent;
    }
    Value number() {
        const auto begin=i_;
        if (i_<s_.size() && s_[i_]=='-') ++i_;
        if (i_>=s_.size() || !detail::digit(s_[i_])) fail("bad number");
        if (s_[i_]=='0') {
            ++i_;
            if (i_<s_.size() && detail::digit(s_[i_])) fail("leading zero");
        } else while (i_<s_.size() && detail::digit(s_[i_])) ++i_;
        if (i_<s_.size() && s_[i_]=='.') {
            ++i_;
            if (i_>=s_.size() || !detail::digit(s_[i_])) fail("missing fractional digits");
            while (i_<s_.size() && detail::digit(s_[i_])) ++i_;
        }
        const auto mantissa_end=i_;
        if (i_<s_.size() && (s_[i_]=='e' || s_[i_]=='E')) {
            ++i_;
            if (i_<s_.size() && (s_[i_]=='+' || s_[i_]=='-')) ++i_;
            if (i_>=s_.size() || !detail::digit(s_[i_])) fail("missing exponent digits");
            while (i_<s_.size() && detail::digit(s_[i_])) ++i_;
        }
        char* end=nullptr;
        const double result=detail::decimal(s_.c_str()+begin,&end);
        if (end!=s_.c_str()+i_ || !std::isfinite(result)) fail("number outside finite double range");
        if (result==0) {
            for (auto j=begin;j<mantissa_end;++j)
                if (s_[j]>='1' && s_[j]<='9') fail("number underflows double range");
        }
        Value value(result);
        value.fractional_lexeme=fractional(begin,mantissa_end);
        return value;
    }
    unsigned hex4() {
        if (s_.size()-i_<4) fail("incomplete unicode escape");
        unsigned code=0;
        for (unsigned j=0;j<4;++j) {
            const char c=s_[i_++];
            unsigned d;
            if (c>='0' && c<='9') d=unsigned(c-'0');
            else if (c>='a' && c<='f') d=unsigned(c-'a'+10);
            else if (c>='A' && c<='F') d=unsigned(c-'A'+10);
            else fail("invalid unicode escape");
            code=(code<<4)|d;
        }
        return code;
    }
    std::string string() {
        if (!eat('"')) fail("expected string");
        std::string out;
        while (i_<s_.size()) {
            const auto c=static_cast<unsigned char>(s_[i_++]);
            if (c=='"') return out;
            if (c<0x20) fail("unescaped control character");
            if (c=='\\') {
                if (i_>=s_.size()) fail("incomplete escape");
                switch (s_[i_++]) {
                case '"': out+='"'; break; case '\\': out+='\\'; break; case '/': out+='/'; break;
                case 'n': out+='\n'; break; case 't': out+='\t'; break; case 'r': out+='\r'; break;
                case 'b': out+='\b'; break; case 'f': out+='\f'; break;
                case 'u': {
                    unsigned code=hex4();
                    if (code>=0xd800 && code<=0xdbff) {
                        if (s_.size()-i_<2 || s_[i_]!='\\' || s_[i_+1]!='u') fail("unpaired high surrogate");
                        i_+=2;
                        const unsigned low=hex4();
                        if (low<0xdc00 || low>0xdfff) fail("invalid low surrogate");
                        code=0x10000+((code-0xd800)<<10)+(low-0xdc00);
                    } else if (code>=0xdc00 && code<=0xdfff) fail("unpaired low surrogate");
                    detail::append_utf8(out,code); break;
                }
                default: fail("invalid escape");
                }
            } else if (c>=0x80) {
                const auto count=detail::utf8_length(s_,i_-1);
                if (!count) fail("invalid UTF-8");
                out.append(s_,i_-1,count); i_+=count-1;
            } else out+=static_cast<char>(c);
        }
        fail("unterminated string");
    }
    Value array(std::size_t depth) {
        eat('['); Array out;
        if (eat(']')) return Value(std::move(out));
        do { out.push_back(value(depth+1)); } while (eat(','));
        if (!eat(']')) fail("expected ]");
        return Value(std::move(out));
    }
    Value object(std::size_t depth) {
        eat('{'); Object out;
        if (eat('}')) return Value(std::move(out));
        do {
            std::string key=string();
            if (!eat(':')) fail("expected :");
            if (out.find(key)!=out.end()) fail("duplicate object key");
            out.emplace(std::move(key),value(depth+1));
        } while (eat(','));
        if (!eat('}')) fail("expected }");
        return Value(std::move(out));
    }
};

inline Value parse(const std::string& text, Limits limits={}) { return Parser(text,limits).parse(); }

inline std::string read_text(std::istream& input, std::size_t limit=Limits{}.max_input_bytes) {
    std::string text;
    char buffer[8192];
    while (true) {
        const auto available=limit-text.size();
        const auto count=available>=sizeof(buffer) ? sizeof(buffer) : available+1;
        input.read(buffer,static_cast<std::streamsize>(count));
        const auto got=static_cast<std::size_t>(input.gcount());
        if (got>available) throw std::runtime_error("json: input byte limit exceeded");
        text.append(buffer,got);
        if (input.bad() || (input.fail() && !input.eof())) throw std::runtime_error("json: input read failed");
        if (input.eof()) return text;
    }
}

// OS diagnostics can contain non-UTF-8 path bytes on POSIX. Preserve those bytes
// visibly as \xHH text rather than letting an error fail JSON serialization.
inline std::string diagnostic(const std::string& message) {
    std::string out;
    static constexpr char hex[]="0123456789abcdef";
    for (std::size_t i=0;i<message.size();) {
        const auto count=detail::utf8_length(message,i);
        if (count) { out.append(message,i,count); i+=count; }
        else {
            const auto c=static_cast<unsigned char>(message[i++]);
            out+="\\x"; out+=hex[c>>4]; out+=hex[c&15];
        }
    }
    return out;
}

inline void write(std::ostream& os, const Value& v, std::size_t depth=0) {
    if (depth>Limits{}.max_depth) throw std::runtime_error("json: output nesting limit exceeded");
    switch (v.kind) {
    case Value::Null: os<<"null"; break;
    case Value::Bool: os<<(v.b?"true":"false"); break;
    case Value::Number: {
        if (!std::isfinite(v.num)) throw std::runtime_error("json: non-finite output number");
        // Preserve the cheap integer path without an out-of-range float cast or
        // locale-dependent grouping. The range check must precede conversion.
        if (std::abs(v.num)<1e15 && std::floor(v.num)==v.num &&
            !(v.num==0 && std::signbit(v.num))) {
            char buffer[32];
            const auto converted=std::to_chars(buffer,buffer+sizeof(buffer),static_cast<long long>(v.num));
            os.write(buffer,static_cast<std::streamsize>(converted.ptr-buffer));
            break;
        }
        std::ostringstream number;
        number.imbue(std::locale::classic());
        number.precision(std::numeric_limits<double>::max_digits10);
        number<<v.num;
        os<<number.str();
        break;
    }
    case Value::String: {
        os<<'"';
        static constexpr char hex[]="0123456789abcdef";
        for (std::size_t i=0;i<v.str.size();) {
            const auto c=static_cast<unsigned char>(v.str[i]);
            if (c>=0x80) {
                const auto count=detail::utf8_length(v.str,i);
                if (!count) throw std::runtime_error("json: invalid UTF-8 output");
                os.write(v.str.data()+i,static_cast<std::streamsize>(count)); i+=count; continue;
            }
            ++i;
            switch (c) {
            case '"': os<<"\\\""; break; case '\\': os<<"\\\\"; break;
            case '\n': os<<"\\n"; break; case '\t': os<<"\\t"; break;
            case '\r': os<<"\\r"; break; case '\b': os<<"\\b"; break; case '\f': os<<"\\f"; break;
            default:
                if (c<0x20) os<<"\\u00"<<hex[c>>4]<<hex[c&15];
                else os.put(static_cast<char>(c));
            }
        }
        os<<'"'; break;
    }
    case Value::ArrayKind: {
        os<<'['; bool first=true;
        for (const auto& e:*v.arr) { if(!first)os<<','; first=false; write(os,e,depth+1); }
        os<<']'; break;
    }
    case Value::ObjectKind: {
        os<<'{'; bool first=true;
        for (const auto& [k,e]:*v.obj) {
            if(!first)os<<',';
            first=false; write(os,Value(k),depth+1); os<<':'; write(os,e,depth+1);
        }
        os<<'}'; break;
    }
    }
}
inline std::string dump(const Value& v) { std::ostringstream ss; write(ss,v); return ss.str(); }
} // namespace uvpack::json
