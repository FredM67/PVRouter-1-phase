// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2024, Benoit BLANCHON
// MIT License

#pragma once

// A meta-function that return the type T if Condition is true.
template< bool Condition, typename T = void >
struct enable_if {};

template< typename T >
struct enable_if<true, T> {
  using type = T;
};

template <bool Condition, typename T = void >
using enable_if_t = typename enable_if< Condition, T >::type;
