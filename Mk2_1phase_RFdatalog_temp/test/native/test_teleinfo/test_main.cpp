/**
 * @file test_main.cpp
 * @brief Native tests for teleinfo.h: the IoT frame, streamed a line per step
 *
 * @details The frame is written by TeleInfo::writeFrameStep(), one line per step of the serial
 *          output (STX before the first line, ETX after the last), from a data source: here, test
 *          sources with every feature set of the firmware. It must give, byte for byte, what the
 *          buffered TeleInfo class of the firmware wrote before (Reference below, its code kept
 *          as it was, utoa()/ultoa() replaced by snprintf()), with the diverted power and energy
 *          (D and E) sent after P when CT2 is fitted.
 */

#include <unity.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "teleinfo.h"  // Real header - pure C++

void setUp(void) {}
void tearDown(void) {}

/** @brief The values of a datalog, shared by every test source. */
struct Values
{
  int16_t power;
  int16_t divertedPower;
  uint16_t divertedEnergy;
  uint16_t voltage_x100;
  int16_t relayAverage;
  bool relayOn[4];
  int16_t loadOnPercent[4];
  int16_t temperature_x100[4];
  bool temperatureValid[4];
  uint16_t absenceCount;
  bool offPeak;
  uint32_t sampleSets;
  uint8_t lowestSampleSetsPerCycle;
};

static Values values;

/** @brief A data source of writeFrameStep(), with a feature set of the firmware. */
template< bool DIVERTED, uint8_t RELAYS, uint8_t LOADS, uint8_t TEMPS, bool DUAL, typename SampleSets = uint16_t >
struct TestSource
{
  static constexpr bool DIVERTED_CT{ DIVERTED };
  static constexpr bool RELAY_DIVERSION{ RELAYS != 0 };
  static constexpr uint8_t RELAY_COUNT{ RELAYS };
  static constexpr uint8_t LOAD_COUNT{ LOADS };
  static constexpr uint8_t TEMP_COUNT{ TEMPS };
  static constexpr bool DUAL_TARIFF{ DUAL };

  static int16_t power() { return values.power; }
  static int16_t divertedPower() { return values.divertedPower; }
  static uint16_t divertedEnergy() { return values.divertedEnergy; }
  static int16_t voltage() { return static_cast< int16_t >(values.voltage_x100); }
  static int16_t relayAverage() { return values.relayAverage; }
  static bool relayOn(uint8_t idx) { return values.relayOn[idx]; }
  static int16_t loadOnPercent(uint8_t idx) { return values.loadOnPercent[idx]; }
  static bool temperatureValid(uint8_t idx) { return values.temperatureValid[idx]; }
  static int16_t temperature(uint8_t idx) { return values.temperature_x100[idx]; }
  static uint16_t absenceCount() { return values.absenceCount; }
  static bool offPeak() { return values.offPeak; }
  static SampleSets sampleSets() { return static_cast< SampleSets >(values.sampleSets); }
  static uint8_t lowestSampleSetsPerCycle() { return values.lowestSampleSetsPerCycle; }
};

/**
 * @brief The buffered TeleInfo class of the firmware, as it was (utoa()/ultoa() aside): the
 *        reference of the frame. Its frame differs in one place only (referenceFrame()): N is
 *        sent unsigned.
 */
class Reference
{
private:
  static const char STX{ 0x02 };
  static const char ETX{ 0x03 };
  static const char LF{ 0x0A };
  static const char CR{ 0x0D };
  static const char TAB{ 0x09 };

  char buffer[512]{};
  size_t bufferPos{ 0 };

  [[nodiscard]] uint8_t calculateChecksum(size_t startPos, size_t endPos) const
  {
    uint8_t sum{ 0 };
    for (const char* ptr = buffer + startPos; ptr != buffer + endPos; ++ptr)
    {
      sum += *ptr;
    }

    return (sum & 0x3F) + 0x20;
  }

  void writeTag(const char* tag, uint8_t index)
  {
    auto* ptr{ tag };
    while (*ptr) buffer[bufferPos++] = *ptr++;

    if (index != 0)
    {
      buffer[bufferPos++] = static_cast< char >('0' + index);
    }

    buffer[bufferPos++] = TAB;
  }

  template< typename Magnitude >
  void writeLine(const char* tag, uint8_t index, Magnitude magnitude, bool negative)
  {
    buffer[bufferPos++] = LF;

    const auto startPos{ bufferPos };

    writeTag(tag, index);
    if (negative)
    {
      buffer[bufferPos++] = '-';
    }
    char* str{ buffer + bufferPos };
    snprintf(str, 12, "%lu", static_cast< unsigned long >(magnitude));  // utoa() / ultoa()
    bufferPos += strlen(str);
    buffer[bufferPos++] = TAB;

    const auto crc{ calculateChecksum(startPos, bufferPos) };
    buffer[bufferPos++] = crc;

    buffer[bufferPos++] = CR;
  }

public:
  void startFrame()
  {
    bufferPos = 0;
    buffer[bufferPos++] = STX;
  }

  void send(const char* tag, int16_t value, uint8_t index = 0)
  {
    const auto magnitude{ value < 0 ? static_cast< uint16_t >(0U - static_cast< uint16_t >(value)) : static_cast< uint16_t >(value) };
    writeLine(tag, index, magnitude, value < 0);
  }

  template< typename Unsigned >
  void sendUnsigned(const char* tag, Unsigned value)
  {
    if constexpr (sizeof(Unsigned) > 2)
    {
      writeLine(tag, 0, static_cast< uint32_t >(value), false);
    }
    else
    {
      writeLine(tag, 0, static_cast< uint16_t >(value), false);
    }
  }

  void endFrame()
  {
    buffer[bufferPos++] = ETX;
  }

  [[nodiscard]] const char* data() const { return buffer; }
  [[nodiscard]] size_t size() const { return bufferPos; }
};

/** @brief The frame of sendTelemetryData() as it was, D and E sent after P with CT2. */
template< typename Source >
static void referenceFrame(Reference& teleInfo)
{
  uint8_t idx{ 0 };

  teleInfo.startFrame();

  teleInfo.send("P", Source::power());

  if constexpr (Source::DIVERTED_CT)
  {
    teleInfo.send("D", Source::divertedPower());
    teleInfo.sendUnsigned("E", Source::divertedEnergy());
  }

  teleInfo.send("V", Source::voltage());

  if constexpr (Source::RELAY_DIVERSION)
  {
    teleInfo.send("R", Source::relayAverage());

    idx = Source::RELAY_COUNT;
    do
    {
      --idx;
      teleInfo.send("R", Source::relayOn(idx), idx + 1);
    } while (idx);
  }

  idx = Source::LOAD_COUNT;
  do
  {
    --idx;
    teleInfo.send("D", Source::loadOnPercent(idx), idx + 1);
  } while (idx);

  for (uint8_t idx = 0; idx < Source::TEMP_COUNT; ++idx)
  {
    if (!Source::temperatureValid(idx))
    {
      continue;
    }
    teleInfo.send("T", Source::temperature(idx), idx + 1);
  }

  teleInfo.sendUnsigned("N", Source::absenceCount());  // unsigned since Task 5b, was send() (negative beyond 32767 s)

  if constexpr (Source::DUAL_TARIFF)
  {
    teleInfo.send("TA", static_cast< int16_t >(Source::offPeak() ? 1 : 0));
  }

  teleInfo.sendUnsigned("S", Source::sampleSets());
  teleInfo.send("S_MC", Source::lowestSampleSetsPerCycle());

  teleInfo.endFrame();
}

/** @brief A frame as the serial output writes it: every step, one after the other. */
struct Streamed
{
  char text[512]{};
  size_t length{ 0 };
  uint8_t steps{ 0 };
  uint8_t longestStep{ 0 };
};

template< typename Source >
static Streamed streamFrame()
{
  Streamed frame;
  bool more{ true };
  while (more)
  {
    char step[TeleInfo::MAX_STEP_LENGTH];
    uint8_t length{ 0 };
    more = TeleInfo::writeFrameStep< Source >(frame.steps++, step, length);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(TeleInfo::MAX_STEP_LENGTH, length);
    memcpy(frame.text + frame.length, step, length);
    frame.length += length;
    if (length > frame.longestStep)
    {
      frame.longestStep = length;
    }
    TEST_ASSERT_LESS_THAN_UINT8(100, frame.steps);  // never endless
  }
  return frame;
}

template< typename Source >
static void assertSameAsReference(const char* message)
{
  Reference reference;
  referenceFrame< Source >(reference);
  const Streamed streamed{ streamFrame< Source >() };

  TEST_ASSERT_EQUAL_UINT32_MESSAGE(reference.size(), streamed.length, message);
  TEST_ASSERT_EQUAL_MEMORY_MESSAGE(reference.data(), streamed.text, reference.size(), message);
}

/** @brief A datalog with every value set, temperature 2 invalid. */
static void typicalValues()
{
  values = Values{ -1234, 1850, 40123, 23045, -567, { true, false, true, false }, { 100, 37, 0, 5 }, { 2137, -12700, 6500, -5 }, { true, false, true, true }, 731, true, 250, 44 };
}

/** @brief The extremes of every value. */
static void extremeValues()
{
  values = Values{ -32768, 32767, 65535, 65535, 32767, { false, true, false, true }, { 0, 100, 1, 99 }, { -32768, 32767, 0, 12500 }, { true, true, true, true }, 32767, false, 65535, 255 };
}

using Minimal = TestSource< false, 0, 1, 0, false >;
using MinimalCT2 = TestSource< true, 0, 1, 0, false >;
using Default = TestSource< false, 0, 3, 0, false >;
using DefaultCT2 = TestSource< true, 0, 3, 0, false >;
using Everything = TestSource< true, 2, 4, 4, true >;
using EverythingNoCT2 = TestSource< false, 2, 4, 4, true >;
using LongPeriod = TestSource< true, 1, 2, 1, false, uint32_t >;

/* the streamed frame is the buffered one, byte for byte, for every feature set */
void test_streamed_frame_equals_buffered(void)
{
  typicalValues();
  assertSameAsReference< Minimal >("minimal, typical");
  assertSameAsReference< Default >("default, typical");
  assertSameAsReference< DefaultCT2 >("default with CT2, typical");
  assertSameAsReference< Everything >("every feature, typical");
  assertSameAsReference< EverythingNoCT2 >("every feature but CT2, typical");

  extremeValues();
  assertSameAsReference< Minimal >("minimal, extremes");
  assertSameAsReference< Default >("default, extremes");
  assertSameAsReference< DefaultCT2 >("default with CT2, extremes");
  assertSameAsReference< Everything >("every feature, extremes");
  assertSameAsReference< EverythingNoCT2 >("every feature but CT2, extremes");

  // every temperature invalid: the temperature steps write nothing
  values.temperatureValid[0] = values.temperatureValid[1] = values.temperatureValid[2] = values.temperatureValid[3] = false;
  assertSameAsReference< Everything >("every feature, no valid temperature");
}

/* beyond 19 s, the sample count is 32 bits */
void test_streamed_frame_32_bit_sample_count(void)
{
  typicalValues();
  values.sampleSets = 4294967295UL;
  assertSameAsReference< LongPeriod >("32-bit sample count, the largest");
  values.sampleSets = 96000;
  assertSameAsReference< LongPeriod >("32-bit sample count, 40 s at 60 Hz");
}

/* a step fits the serial output's (SerialOutput::MAX_STEP_LENGTH, 32): one line, STX or ETX */
void test_step_fits_serial_output(void)
{
  extremeValues();
  // 32: SerialOutput::MAX_STEP_LENGTH, out of reach here (serial_output.h needs Arduino.h);
  // utils.h checks it with a static_assert, the embedded test against the constant
  TEST_ASSERT_LESS_OR_EQUAL_UINT8(32, TeleInfo::MAX_STEP_LENGTH);
  TEST_ASSERT_LESS_OR_EQUAL_UINT8(TeleInfo::MAX_STEP_LENGTH, streamFrame< Everything >().longestStep);

  values.sampleSets = 4294967295UL;
  TEST_ASSERT_LESS_OR_EQUAL_UINT8(TeleInfo::MAX_STEP_LENGTH, streamFrame< LongPeriod >().longestStep);
}

/* one step per line: P, V, D1, N, S and S_MC */
void test_one_line_per_step(void)
{
  typicalValues();
  TEST_ASSERT_EQUAL_UINT8(6, streamFrame< Minimal >().steps);
  TEST_ASSERT_EQUAL_UINT8(8, streamFrame< MinimalCT2 >().steps);
  // P D E V R R2 R1 D4..D1 T1..T4 N TA S S_MC
  TEST_ASSERT_EQUAL_UINT8(19, streamFrame< Everything >().steps);
}

/* a frame written independently of the code, checksums computed by hand */
void test_golden_frame(void)
{
  values = Values{};
  values.power = -25;
  values.voltage_x100 = 23012;
  values.loadOnPercent[0] = 40;
  values.absenceCount = 7;
  values.sampleSets = 250;
  values.lowestSampleSetsPerCycle = 44;

  // checksum: the sum of the bytes from the tag to the second TAB, & 0x3F, + 0x20
  // P\t-25\t     0x0F6 -> 0x36 + 0x20 = 'V'
  // V\t23012\t   0x160 -> 0x20 + 0x20 = '@'
  // D1\t40\t     0x0EB -> 0x2B + 0x20 = 'K'
  // N\t7\t       0x097 -> 0x17 + 0x20 = '7'
  // S\t250\t     0x0FC -> 0x3C + 0x20 = '\'
  // S_MC\t44\t   0x1BC -> 0x3C + 0x20 = '\'
  static const char golden[]{ "\x02"
                              "\nP\t-25\tV\r"
                              "\nV\t23012\t@\r"
                              "\nD1\t40\tK\r"
                              "\nN\t7\t7\r"
                              "\nS\t250\t\\\r"
                              "\nS_MC\t44\t\\\r"
                              "\x03" };

  const Streamed streamed{ streamFrame< Minimal >() };
  TEST_ASSERT_EQUAL_UINT32(sizeof golden - 1, streamed.length);
  TEST_ASSERT_EQUAL_MEMORY(golden, streamed.text, sizeof golden - 1);
}

/** @brief A line of a frame: its tag (with the index) and its value. */
struct Line
{
  char tag[8];
  char value[16];
};

/**
 * @brief Splits a frame into its lines, as a receiver does, checking STX, ETX and every
 *        line's format and checksum.
 * @return the number of lines
 */
static uint8_t parseFrame(const Streamed& frame, Line (&lines)[32])
{
  TEST_ASSERT_GREATER_THAN_UINT32(2, frame.length);
  TEST_ASSERT_EQUAL_CHAR(0x02, frame.text[0]);
  TEST_ASSERT_EQUAL_CHAR(0x03, frame.text[frame.length - 1]);

  uint8_t count{ 0 };
  size_t pos{ 1 };
  while (pos < frame.length - 1)
  {
    TEST_ASSERT_EQUAL_CHAR('\n', frame.text[pos++]);
    const size_t start{ pos };
    const char* const tab1{ static_cast< const char* >(memchr(frame.text + pos, '\t', frame.length - pos)) };
    TEST_ASSERT_NOT_NULL(tab1);
    const char* const tab2{ static_cast< const char* >(memchr(tab1 + 1, '\t', frame.length - (tab1 + 1 - frame.text))) };
    TEST_ASSERT_NOT_NULL(tab2);

    uint8_t sum{ 0 };
    for (const char* p = frame.text + start; p != tab2 + 1; ++p)
    {
      sum += static_cast< uint8_t >(*p);
    }
    TEST_ASSERT_EQUAL_CHAR((sum & 0x3F) + 0x20, tab2[1]);
    TEST_ASSERT_EQUAL_CHAR('\r', tab2[2]);

    Line& line{ lines[count++] };
    memset(&line, 0, sizeof line);
    memcpy(line.tag, frame.text + start, tab1 - (frame.text + start));
    memcpy(line.value, tab1 + 1, tab2 - (tab1 + 1));
    pos = (tab2 + 3) - frame.text;
  }
  return count;
}

/** @brief How many lines have exactly this tag. */
static uint8_t countTag(const Line (&lines)[32], uint8_t count, const char* tag)
{
  uint8_t found{ 0 };
  for (uint8_t i = 0; i != count; ++i)
  {
    found += strcmp(lines[i].tag, tag) == 0 ? 1 : 0;
  }
  return found;
}

/** @brief The value of the line with exactly this tag. */
static const char* valueOf(const Line (&lines)[32], uint8_t count, const char* tag)
{
  for (uint8_t i = 0; i != count; ++i)
  {
    if (strcmp(lines[i].tag, tag) == 0)
    {
      return lines[i].value;
    }
  }
  return "";
}

/* N, the seconds without diverted energy, is unsigned: beyond 32767 s (9.1 h) it no longer
 * reads negative */
void test_absence_count_unsigned(void)
{
  typicalValues();
  Line lines[32];

  values.absenceCount = 40000;
  uint8_t count{ parseFrame(streamFrame< Default >(), lines) };
  TEST_ASSERT_EQUAL_STRING("40000", valueOf(lines, count, "N"));
  assertSameAsReference< Default >("N = 40000");
  assertSameAsReference< Everything >("every feature, N = 40000");

  values.absenceCount = 65535;
  count = parseFrame(streamFrame< Everything >(), lines);
  TEST_ASSERT_EQUAL_STRING("65535", valueOf(lines, count, "N"));
  assertSameAsReference< Everything >("every feature, N = 65535");
}

/* with CT2: D (W) and E (Wh) right after P, apart from D1...Dn (the loads' ON time, %) */
void test_diverted_labels_with_ct2(void)
{
  typicalValues();
  values.divertedPower = 1850;
  values.divertedEnergy = 40123;  // above 32767: unsigned
  values.loadOnPercent[0] = 100;
  values.loadOnPercent[1] = 37;
  values.loadOnPercent[2] = 0;

  Line lines[32];
  const uint8_t count{ parseFrame(streamFrame< DefaultCT2 >(), lines) };

  TEST_ASSERT_EQUAL_STRING("P", lines[0].tag);
  TEST_ASSERT_EQUAL_STRING("D", lines[1].tag);
  TEST_ASSERT_EQUAL_STRING("1850", lines[1].value);
  TEST_ASSERT_EQUAL_STRING("E", lines[2].tag);
  TEST_ASSERT_EQUAL_STRING("40123", lines[2].value);
  TEST_ASSERT_EQUAL_UINT8(1, countTag(lines, count, "D"));
  TEST_ASSERT_EQUAL_UINT8(1, countTag(lines, count, "E"));

  // the loads' ON time, still there and untouched
  TEST_ASSERT_EQUAL_STRING("100", valueOf(lines, count, "D1"));
  TEST_ASSERT_EQUAL_STRING("37", valueOf(lines, count, "D2"));
  TEST_ASSERT_EQUAL_STRING("0", valueOf(lines, count, "D3"));
  TEST_ASSERT_EQUAL_UINT8(1, countTag(lines, count, "D1"));
  TEST_ASSERT_EQUAL_UINT8(1, countTag(lines, count, "D2"));
  TEST_ASSERT_EQUAL_UINT8(1, countTag(lines, count, "D3"));

  // every feature: still a single D and a single E, R and T apart
  const uint8_t all{ parseFrame(streamFrame< Everything >(), lines) };
  TEST_ASSERT_EQUAL_UINT8(1, countTag(lines, all, "D"));
  TEST_ASSERT_EQUAL_STRING("1850", valueOf(lines, all, "D"));
  TEST_ASSERT_EQUAL_UINT8(1, countTag(lines, all, "E"));
  TEST_ASSERT_EQUAL_STRING("40123", valueOf(lines, all, "E"));
  TEST_ASSERT_EQUAL_UINT8(1, countTag(lines, all, "D4"));
}

/* without CT2: neither D nor E, the loads' D1...Dn still there */
void test_no_diverted_labels_without_ct2(void)
{
  typicalValues();

  Line lines[32];
  const uint8_t count{ parseFrame(streamFrame< Default >(), lines) };

  TEST_ASSERT_EQUAL_UINT8(0, countTag(lines, count, "D"));
  TEST_ASSERT_EQUAL_UINT8(0, countTag(lines, count, "E"));
  TEST_ASSERT_EQUAL_STRING("P", lines[0].tag);
  TEST_ASSERT_EQUAL_STRING("V", lines[1].tag);
  TEST_ASSERT_EQUAL_UINT8(1, countTag(lines, count, "D1"));
  TEST_ASSERT_EQUAL_UINT8(1, countTag(lines, count, "D2"));
  TEST_ASSERT_EQUAL_UINT8(1, countTag(lines, count, "D3"));

  const uint8_t all{ parseFrame(streamFrame< EverythingNoCT2 >(), lines) };
  TEST_ASSERT_EQUAL_UINT8(0, countTag(lines, all, "D"));
  TEST_ASSERT_EQUAL_UINT8(0, countTag(lines, all, "E"));
  TEST_ASSERT_EQUAL_UINT8(1, countTag(lines, all, "D4"));
}

/* a single line: LF, tag and index, TAB, value, TAB, checksum, CR */
void test_line(void)
{
  char text[TeleInfo::MAX_STEP_LENGTH];

  // T1\t-5500\t: 0x18E -> 0x0E + 0x20 = '.'
  const char* end{ TeleInfo::line(text, "T", -5500, 1) };
  TEST_ASSERT_EQUAL_UINT32(12, end - text);
  TEST_ASSERT_EQUAL_MEMORY("\nT1\t-5500\t.\r", text, 12);

  // E\t65535: unsigned, not -1
  end = TeleInfo::lineUnsigned(text, "E", static_cast< uint16_t >(65535));
  TEST_ASSERT_EQUAL_UINT32(11, end - text);
  TEST_ASSERT_EQUAL_MEMORY("\nE\t65535\t", text, 9);

  // the longest: a 4-character tag and 10 digits
  end = TeleInfo::lineUnsigned(text, "S_MC", static_cast< uint32_t >(4294967295UL));
  TEST_ASSERT_EQUAL_UINT32(TeleInfo::lineSize(4, 10), end - text);
  TEST_ASSERT_EQUAL_UINT8(TeleInfo::MAX_STEP_LENGTH - 1, TeleInfo::lineSize(4, 10));

  // 0 and -32768
  end = TeleInfo::line(text, "P", 0);
  TEST_ASSERT_EQUAL_MEMORY("\nP\t0\t", text, 5);
  end = TeleInfo::line(text, "P", -32768);
  TEST_ASSERT_EQUAL_MEMORY("\nP\t-32768\t", text, 10);
}

int main(int /* argc */, char** /* argv */)
{
  UNITY_BEGIN();

  RUN_TEST(test_line);
  RUN_TEST(test_golden_frame);
  RUN_TEST(test_streamed_frame_equals_buffered);
  RUN_TEST(test_streamed_frame_32_bit_sample_count);
  RUN_TEST(test_step_fits_serial_output);
  RUN_TEST(test_one_line_per_step);
  RUN_TEST(test_absence_count_unsigned);
  RUN_TEST(test_diverted_labels_with_ct2);
  RUN_TEST(test_no_diverted_labels_without_ct2);

  return UNITY_END();
}
