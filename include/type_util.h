#pragma once

#include <string_view>

template<typename T>
constexpr auto full_type_name() noexcept {
    std::string_view name, prefix, suffix;
#ifdef __clang__
    name = __PRETTY_FUNCTION__;
    prefix = "auto full_type_name() [T = ";
    suffix = "]";
#elif defined(__GNUC__)
    name = __PRETTY_FUNCTION__;
    prefix = "constexpr auto full_type_name() [with T = ";
    suffix = "]";
#elif defined(_MSC_VER)
    name = __FUNCSIG__;
    prefix = "auto __cdecl full_type_name<";
    suffix = ">(void) noexcept";
#endif
    name.remove_prefix(prefix.size());
    name.remove_suffix(suffix.size());
    return name;
}


template<typename T>
constexpr auto full_type_name_str() noexcept {
    return std::string(full_type_name<T>());
}




constexpr std::string_view type_name_without_ns(std::string_view full_name) noexcept {
    auto final_part_pos = full_name.rfind("::");
    if (final_part_pos == std::string_view::npos) {
        return full_name;
    } else {
        return full_name.substr(final_part_pos + 2);
    }
}



template<class ...C>
auto template_name() {
    std::string tt((std::string(full_type_name<C>()) + "," +...));
    tt.erase(tt.size() - 1);
    tt = '<' + tt + '>';
    return tt;
}

template<>
auto template_name() {
    return "";
}

template<class ...C>
auto append_template_name(const std::string &name) {
    return name + template_name<C...>();
}

#define REGISTER_NAME(name, ...) append_template_name<__VA_ARGS__>(name)


namespace spiritsaway::entity_component_event {


template<typename B>
class base_type_hash {
    static std::size_t last_used_id;
public:
    template<typename T>
    static std::enable_if_t<std::is_base_of_v<B, T>, std::size_t> hash() {
        static const std::size_t id = last_used_id++;
        return id;
    }

    static std::size_t max_used() {
        return last_used_id;
    }
};

template<typename B>
std::size_t base_type_hash<B>::last_used_id = 0;
}
