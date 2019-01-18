//
// Created by Jonathan Ohayon on 2019-01-18.
//

#include "utils.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
