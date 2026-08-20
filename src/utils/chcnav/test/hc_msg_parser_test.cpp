#include <gtest/gtest.h>

#include <cstddef>

extern "C"
{
#include "hc_msg_parser.h"
}

namespace
{

struct EmptySource
{
    int read_count = 0;
};

int readNoData(void *, std::size_t, std::size_t *size_read, void *parser_id)
{
    EmptySource *source = static_cast<EmptySource *>(parser_id);
    ++source->read_count;
    *size_read = 0U;
    return 0;
}

TEST(HcMsgParser, EmptyReadReturnsNoDataWithoutInternalRetry)
{
    EmptySource source;
    hc__msg_parser_t parser{};
    hc__msg_token_t token{};

    ASSERT_EQ(0, hc__init_msg_parser(&parser, 64U, readNoData, &source));

    EXPECT_EQ(HC__SCAN_NO_DATA, hc__msg_parser_scan(&parser, &token));
    EXPECT_EQ(1, source.read_count);

    hc__deinit_msg_parser(&parser);
}

} // namespace
