#pragma once
#include <limits>

//Costante positiva piccola
template<typename T>
constexpr T EPSILON = std::numeric_limits<T>::epsilon();
//----------------------------------------------------------------------------------
//Almost predicates(confronti in senso lato)
//Valore minore o all'incirca uguale
template<typename T>
constexpr bool almostLower(T lhs, T rhs) noexcept
{
    return lhs <= rhs + EPSILON<T>;
}
//Valore maggiore o all'incirca uguale
template<typename T>
constexpr bool almostGreater(T lhs, T rhs) noexcept
{
    return lhs >= rhs - EPSILON<T>;
}
//Valori sostanzialmente uguali
template<typename T>
constexpr bool almostEqual(T lhs, T rhs) noexcept
{   //valgono contemporaneamente almostLower e almostGreater
    return almostLower(lhs, rhs) && almostGreater(lhs, rhs);
}
//Valore all'incirca uguale a zero
template<typename T>
constexpr bool almostZero(T x) noexcept
{
    return almostEqual(x, static_cast<T>(0));
}
//Valore x all'incirca compreso tra a e b
template<typename T>
constexpr bool almostBetween(T x, T a, T b) noexcept
{
    return almostGreater(x, a) && almostLower(x, b);
}
//----------------------------------------------------------------------------------
//Strictly predicates(confronti in senso stretto)
//Valore strettamente minore
template<typename T>
constexpr bool strictlyLower(T lhs, T rhs) noexcept
{
    return lhs < rhs - EPSILON<T>;
}
//Valore strettamente maggiore
template<typename T>
constexpr bool strictlyGreater(T lhs, T rhs) noexcept
{
    return lhs > rhs + EPSILON<T>;
}
//Valore x strettamente compreso tra a e b
template<typename T>
constexpr bool strictlyBetween(T x, T a, T b) noexcept
{
    return strictlyGreater(x, a) && strictlyLower(x, b);
}
