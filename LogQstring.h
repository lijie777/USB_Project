#ifndef LOGQSTRING_H
#define LOGQSTRING_H

#include <QString>
#include "spdlog/fmt/bundled/core.h"

// 为QString添加fmt格式化支持
template <>
struct fmt::formatter<QString> {
    constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin())
    {
        return ctx.begin();
    }

    template <typename FormatContext>
    auto format(const QString& str, FormatContext& ctx) -> decltype(ctx.out())
    {
        return format_to(ctx.out(), "{}", str.toStdString());
    }
};

#endif  // LOGQSTRING_H
