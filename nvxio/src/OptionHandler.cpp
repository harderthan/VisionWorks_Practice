/*
# Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cassert>
#include <limits>
#include <sstream>

#include "NVXIO/Application.hpp"
#include "NVXIO/Utility.hpp"

namespace nvxio {

namespace {

class OptionHandlerString : public OptionHandler
{
public:
    explicit OptionHandlerString(std::string *result) : result(result)
    {
        defaultString = std::string("\"") + *result + std::string("\"");
    }

    std::string getPlaceholder() const { return "STRING"; }

    std::string getDefaultString() const { return defaultString; }

    std::string processValue(const std::string &valueStr) const {
        *result = valueStr;
        return std::string();
    }

private:
    std::string *result;
    std::string defaultString;
};

template <typename T>
std::string representRange(const Range<T> &range)
{
    if (!range.lowConstrained() && !range.highConstrained()) return "";

    std::ostringstream osLow, osHigh;
    osLow << (range.lowInclusive ? ">=" : ">") << ' ' << range.low;
    osHigh << (range.highInclusive ? "<=" : "<") << ' ' << range.high;

    if (!range.lowConstrained())
        return osHigh.str();
    else if (!range.highConstrained())
        return osLow.str();
    else
        return osLow.str() + " and " + osHigh.str();
}

template <typename T>
class OptionHandlerStreamBased : public OptionHandler
{
public:
    OptionHandlerStreamBased(T *result, const Range<T> &validRange)
        : result(result), validRange(validRange)
    {
        assert(validRange.contains(*result));
        std::ostringstream os;
        os << *result;
        defaultString = os.str();
    }

    std::string getDefaultString() const {
        return defaultString;
    }

    std::string processValue(const std::string &valueStr) const {
        std::istringstream is(valueStr);
        std::noskipws(is);
        is >> *result;

        if (!is || is.peek() != EOF) return getParseErrorString();

        // The condition here looks more complicated than it needs to be, but
        // it's deliberately written this way so that NaN would fail the test.
        if (!(*result >= std::numeric_limits<T>::lowest()
                && *result <= std::numeric_limits<T>::max()))
            return "must be a finite value";

        if (!validRange.contains(*result))
            return getConstraintString();

        return "";
    }

    std::string getConstraintString() const {
        std::string rangeStr = representRange(validRange);
        if (rangeStr.empty()) return rangeStr;
        return "must be " + rangeStr;
    }

protected:
    virtual std::string getParseErrorString() const = 0;

private:
    T *result;
    Range<T> validRange;
    std::string defaultString;
};

class OptionHandlerInteger : public OptionHandlerStreamBased<int>
{
public:
    OptionHandlerInteger(int *result, const Range<int> &validRange)
        : OptionHandlerStreamBased<int>(result, validRange)
    {}

    std::string getPlaceholder() const { return "INTEGER"; }

protected:
    std::string getParseErrorString() const { return "must be an integer"; }
};

class OptionHandlerUnsignedInteger : public OptionHandlerStreamBased<unsigned>
{
public:
    OptionHandlerUnsignedInteger(unsigned *result, const Range<unsigned> &validRange)
        : OptionHandlerStreamBased<unsigned>(result, validRange)
    {}

    std::string processValue(const std::string &valueStr) const
    {
        // istream's operator >>(unsigned) doesn't report an error on negative
        // numbers, so we have to check ourselves.
        if (!valueStr.empty() && valueStr.front() == '-')
            return getParseErrorString();
        return OptionHandlerStreamBased<unsigned>::processValue(valueStr);
    }

    std::string getPlaceholder() const { return "INTEGER"; }

protected:
    std::string getParseErrorString() const { return "must be a non-negative integer"; }
};

template <typename T>
class OptionHandlerReal : public OptionHandlerStreamBased<T>
{
public:
    OptionHandlerReal(T *result, const Range<T> &validRange)
        : OptionHandlerStreamBased<T>(result, validRange)
    {}

    std::string getPlaceholder() const { return "REAL"; }

protected:
    std::string getParseErrorString() const { return "must be a real number"; }
};

}

OptionHandler::~OptionHandler()
{}

std::string OptionHandler::getConstraintString() const
{
    return std::string();
}

OptionHandler::ptr OptionHandler::string(std::string *result)
{
    return makeUP<OptionHandlerString>(result);
}

OptionHandler::ptr OptionHandler::integer(int *result, const Range<int> &validRange)
{
    return makeUP<OptionHandlerInteger>(result, validRange);
}

OptionHandler::ptr OptionHandler::unsignedInteger(unsigned *result, const Range<unsigned> &validRange)
{
    return makeUP<OptionHandlerUnsignedInteger>(result, validRange);
}

OptionHandler::ptr OptionHandler::real(float *result, const Range<float> &validRange)
{
    return makeUP<OptionHandlerReal<float>>(result, validRange);
}

OptionHandler::ptr OptionHandler::real(double *result, const Range<double> &validRange)
{
    return makeUP<OptionHandlerReal<double>>(result, validRange);
}

OptionHandler::ptr OptionHandler::oneOf(std::string *result, std::initializer_list<std::string> allowedValues)
{
    std::vector<std::pair<std::string, std::string>> allowedPairs;
    allowedPairs.reserve(allowedValues.size());
    for (const auto &v: allowedValues) allowedPairs.emplace_back(v, v);

    return makeUP<detail::OptionHandlerOneOf<std::string>>(result, allowedPairs.begin(), allowedPairs.end());
}

}
