#ifndef BITFIELD_H
#define BITFIELD_H

#include <algorithm>
#include <cstdint>
#include <type_traits>
#include <utility>

/* Template support for strongly typed bit fields for registers etc. To use: Declare an enum class
 * with the values as bit numbers (e.g OPTION_IN_BIT5 = 5, SOME_OTHER_OPTION = 7). To declare a
 * field with width, use "SOME_MULTIBIT_OPTION = field(bit_number, width)". Then use
 * DECLARE_BITFIELD on the enum class to enable the template. To enter/extract numbers from wide
 * fields you can use the << and >> operators. You can also use DECLARE_BITFIELD_OPTIONS to set
 * another enum class as the list of possible values for a field. The those options can be combined
 * with the field using |. The wide field name alone behaves as if filled with ones to facilitate
 * use as a mask. Setting an OPTION with | or |= clears its respective field by magic. Using << to
 * set a field does not. Otherwise using &, ~, and | behaves as you would expect for a bit field of
 * integers, where OPTION_IN_BIT5 is (1<<5).
 */
template <class T>
constexpr bool is_enum_class = !std::is_convertible<T, int>::value && std::is_enum<T>::value;


namespace utils {
template <typename>
struct bitfield_enum_types {};
template<class T, class U>
struct bitfield_options_t2 {}; //MSVC workaround
template <class T, class U>
struct bitfield_options_t {};
}

#define DECLARE_BITFIELD(ENUM, INT)                                                                \
    namespace utils {                                                                              \
    template <>                                                                                    \
    struct bitfield_enum_types<ENUM> {                                                             \
    typedef ENUM Enum;                                                                         \
    typedef INT Int;                                                                           \
    };                                                                                             \
    }
template <typename T>
using bitfield_enum_t = typename utils::bitfield_enum_types<T>::Enum;
template <typename T>
using bitfield_int_t = typename utils::bitfield_enum_types<T>::Int;

#define DECLARE_BITFIELD_OPTIONS(FIELD, OPTIONS)                                                   \
    namespace utils {                                                                              \
    template <>                                                                                    \
    struct bitfield_options_t<OPTIONS, decltype(FIELD)> {                                          \
    constexpr static auto field = FIELD;                                                       \
    typedef bitfield<decltype(FIELD)> bitfield_type;                                           \
    };                                                                                             \
    template <>                                                                                    \
    struct bitfield_options_t2<OPTIONS, decltype(FIELD)> {                                         \
    typedef OPTIONS options_type;                                                              \
    };                                                                                             \
    }

// Get number from enum class
template <typename Enum>
inline std::enable_if_t<is_enum_class<Enum>, std::underlying_type_t<Enum>> num(Enum e) {
    return static_cast<std::underlying_type_t<Enum>>(e);
}

template <typename Enum, bitfield_enum_t<Enum>...>
inline auto bit(Enum e) {
    return num(e) & 0xFF;
}

template <typename Enum, bitfield_enum_t<Enum>...>
inline auto width(Enum e) {
    return (num(e) >> 8) + 1;
}

constexpr auto field(unsigned bit, unsigned width) { return ((width - 1) << 8) | bit; }

// Convert from enum classes defining bit numbers to bit fields
template <typename Enum_>
class bitfield {
    using Enum = bitfield_enum_t<Enum_>;
public:
    using Int = bitfield_int_t<Enum_>;
private:
    Int f;

public:
    bitfield() : f(0) {}
    bitfield(Enum e) : f(((bitfield_int_t<Enum>(1) << width(e)) - 1) << bit(e)) {}
    bitfield(const bitfield<Enum>& a) : f(a.f) {}
    bitfield(const volatile bitfield<Enum>& a) : f(a.f) {}
    bitfield(bitfield<Enum>&& a) : f(a.f) {}
    bitfield<Enum>& operator=(const bitfield<Enum>& a) {
        f = a.f;
        return *this;
    }
    void operator=(const bitfield<Enum>& a) volatile { f = a.f; }
    void operator=(const volatile bitfield<Enum>& a) volatile { f = a.f; }
    bitfield<Enum>& operator=(bitfield<Enum>&& a) {
        f = a.f;
        return *this;
    }
    void operator=(bitfield<Enum>&& a) volatile { f = a.f; }
    explicit bitfield(Int f) : f(f) {}
    template <typename T> // Disable any other implict conversion
    operator T() const = delete;
    template <typename T>
    operator T() const volatile = delete;
    explicit operator Int() const { return f; }
    operator bool() const { return f > 0; }
    explicit operator Int() const volatile { return f; }
    operator bool() const volatile { return f > 0; }

    friend inline bitfield<Enum> operator|(bitfield<Enum> a, bitfield<Enum> b) {
        return bitfield<Enum>(a.f | b.f);
    }

    friend inline bitfield<Enum> operator&(bitfield<Enum> a, bitfield<Enum> b) {
        return bitfield<Enum>(a.f & b.f);
    }

    friend inline bitfield<Enum> operator|=(bitfield<Enum>& a, bitfield<Enum> b) {
        a.f |= b.f;
        return a;
    }

    friend inline bitfield<Enum> operator&=(bitfield<Enum>& a, bitfield<Enum> b) {
        a.f &= b.f;
        return a;
    }

    friend inline void operator|=(volatile bitfield<Enum>& a, bitfield<Enum> b) { a.f |= b.f; }

    friend inline void operator&=(volatile bitfield<Enum>& a, bitfield<Enum> b) { a.f &= b.f; }

    friend inline bitfield<Enum> operator~(bitfield<Enum> b) { return bitfield<Enum>(~b.f); }

    friend inline bool operator==(bitfield<Enum> a, bitfield<Enum> b) { return a.f == b.f; }

    friend inline bool operator!=(bitfield<Enum> a, bitfield<Enum> b) { return a.f != b.f; }

    //MSVC requires bitfield_options_t and bitfield_options_t2 for some bizarre reason
    template <typename Option, typename utils::bitfield_options_t2<Option, Enum>::options_type...>
    explicit bitfield(Option o)
        : f((static_cast<Int>(o) << bit(utils::bitfield_options_t<Option, Enum>::field)) &
            bitfield<Enum>(utils::bitfield_options_t<Option, Enum>::field).f) {}

    template <typename Option, typename utils::bitfield_options_t2<Option, Enum>::options_type...>
    bitfield<Enum>& operator=(Option a) {
        f = bitfield<Enum>(a);
        return *this;
    }
    template <typename Option, typename utils::bitfield_options_t2<Option, Enum>::options_type...>
    void operator=(Option a) volatile {
        f = bitfield<Enum>(a);
    }

    template <typename Option, typename utils::bitfield_options_t<Option, Enum>::bitfield_type...>
    friend inline bitfield<Enum> operator|(bitfield<Enum> a, Option b) {
        return (a & ~bitfield<Enum>(utils::bitfield_options_t<Option, Enum>::field)) |
                bitfield<Enum>(b);
    }
    template <typename Option, typename utils::bitfield_options_t<Option, Enum>::bitfield_type...>
    friend inline bitfield<Enum> operator|(Option a, bitfield<Enum> b) {
        return b | a;
    }
    template <typename Option, typename utils::bitfield_options_t<Option, Enum>::bitfield_type...>
    friend inline bitfield<Enum> operator|=(bitfield<Enum>& a, Option b) {
        a = a | b;
        return a;
    }
    template <typename Option, typename utils::bitfield_options_t<Option, Enum>::bitfield_type...>
    friend inline void operator|=(volatile bitfield<Enum>& a, Option b) {
        a = a | b;
    }
    template <typename Option, typename utils::bitfield_options_t<Option, Enum>::bitfield_type...>
    friend inline bool operator==(bitfield<Enum> a, Option b) {
        return a == bitfield<Enum>(b);
    }
    template <typename Option, typename utils::bitfield_options_t<Option, Enum>::bitfield_type...>
    friend inline bool operator==(Option a, bitfield<Enum> b) {
        return b == a;
    }
    template <typename Option, typename utils::bitfield_options_t<Option, Enum>::bitfield_type...>
    friend inline bool operator!=(bitfield<Enum> a, Option b) {
        return a != bitfield<Enum>(b);
    }
    template <typename Option, typename utils::bitfield_options_t<Option, Enum>::bitfield_type...>
    friend inline bool operator!=(Option a, bitfield<Enum> b) {
        return b != a;
    }
};

template <typename Enum>
Enum first(bitfield<Enum> bf) {
    return static_cast<Enum>(first(bitfield_int_t<Enum>(bf)));
}

// Create bitfield from enum class
template <typename Enum, bitfield_enum_t<Enum>...>
inline auto bf(Enum e) {
    return bitfield<Enum>(e);
}

template <typename Enum, bitfield_enum_t<Enum>...>
inline auto operator|(Enum a, Enum b) {
    return bf(a) | b;
}

template <typename Enum, typename Option,
          typename utils::bitfield_options_t<Option, Enum>::bitfield_type...>
inline auto operator|(Enum a, Option b) {
    return bf(a) | b;
}

template <typename Enum, typename Option,
          typename utils::bitfield_options_t<Option, Enum>::bitfield_type...>
inline auto operator|(Option b, Enum a) {
    return bf(a) | b;
}

template <typename Enum, bitfield_enum_t<Enum>...>
inline auto operator&(Enum a, Enum b) {
    return bf(a == b ? a : 0);
}

template <typename Enum>
inline auto operator<<(bitfield_int_t<Enum> i, Enum e) {
    return bitfield<Enum>(i << bit(e)) & bitfield<Enum>(e);
}

template <typename Enum>
inline auto operator>>(bitfield<Enum> b, Enum e) {
    return bitfield_int_t<Enum>(b & bitfield<Enum>(e)) >> bit(e);
}

template <typename Enum, bitfield_enum_t<Enum>...>
inline auto operator~(Enum e) {
    return ~bitfield<Enum>(e);
}

#endif // UTILS_H
