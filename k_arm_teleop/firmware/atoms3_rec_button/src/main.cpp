// Record light and switch of the k_arm teleop (rosbag), on an AtomS3.
//
// Line protocol over the USB serial, one message per line:
//
//   PC -> AtomS3
//     S rec=<0|1> dur=<s> size=<bytes> bags=<n> phase=<phase> target=<target> grip=<open|closed|->
//         teleop status, about 5 Hz (rec_button_bridge.py); grip: the gripper
//         set by HOLD in place of the trigger, - when the trigger drives it
//     S none             the bridge runs but hears no teleop manager
//     R ok               the last CLICK / HOLD did what it asked
//     R err <reason>     it did not
//   AtomS3 -> PC
//     HELLO k_arm_rec_button board=<board>   at boot and on "?"
//     CLICK              the screen (button) was pressed briefly: start / stop recording
//     HOLD               it was held HOLD_MS: open <-> close the real gripper
//
// Screens: REC (red), STANDBY (dark), NOT REC (orange blink: following but
// not recording), NO TELEOP ("S none"), NO LINK (no line from the PC for
// LINK_TIMEOUT_MS). While recording, NO LABEL warns that teleop is not
// following (/k_arm_teleop_manager/command is only published while
// following), VIRTUAL that the real robot is not moving; neither is shown
// with phase=record_only (launch/record_only.launch: recording without the
// teleop, so there is no label and no target to warn about).

#include <M5Unified.h>

namespace
{
constexpr uint32_t LINK_TIMEOUT_MS = 2000;     // no line from the PC for this long -> NO LINK
constexpr uint32_t REPLY_TIMEOUT_MS = 5000;    // CLICK without "R ..." for this long -> NO REPLY
constexpr uint32_t HOLD_MS = 800;              // a press this long is HOLD, not CLICK
constexpr uint32_t RESULT_SHOW_MS = 1500;      // how long "R ..." stays on the screen
constexpr uint32_t BLINK_PERIOD_MS = 400;
constexpr uint32_t FRAME_PERIOD_MS = 50;
constexpr size_t MAX_LINE = 160;

constexpr uint16_t RED = 0xC904;      // #c62828, as the panel's record button
constexpr uint16_t ORANGE = 0xEB60;   // #ef6c00, as the panel's warnings
constexpr uint16_t DARK = 0x10A2;
constexpr uint16_t GREY = 0x4208;
constexpr uint16_t YELLOW = 0xFFE0;
constexpr uint16_t WHITE = 0xFFFF;
constexpr uint16_t BLACK = 0x0000;

struct Status
{
  bool manager = false;   // false after "S none"
  bool recording = false;
  float duration = 0.0f;
  uint64_t size = 0;
  long bags = -1;          // -1: not sent
  String phase;
  String target;
  String grip;             // "open" / "closed" set by HOLD, "-" or empty: the trigger
};

M5Canvas canvas(&M5.Display);
Status status;
uint32_t last_line_ms = 0;
bool have_line = false;
bool waiting_reply = false;
uint32_t click_ms = 0;
String result;            // "OK" or the error of the last "R ..."
bool result_ok = false;
uint32_t result_ms = 0;
char line[MAX_LINE + 1];
size_t line_len = 0;
bool line_overflow = false;

const char* boardName()
{
  switch (M5.getBoard())
  {
    case m5::board_t::board_M5AtomS3:
      return "AtomS3";
    case m5::board_t::board_M5AtomS3Lite:
      return "AtomS3Lite";
    case m5::board_t::board_M5AtomS3R:
      return "AtomS3R";
    default:
      return "unknown";
  }
}

void hello()
{
  Serial.printf("HELLO k_arm_rec_button board=%s\n", boardName());
}

// value of "key=" in a status line; empty if missing
String field(const String& text, const char* key)
{
  String pattern = String(" ") + key + "=";
  int start = text.indexOf(pattern);
  if (start < 0)
    return String();
  start += pattern.length();
  int end = text.indexOf(' ', start);
  return end < 0 ? text.substring(start) : text.substring(start, end);
}

void handleLine(const String& text)
{
  last_line_ms = millis();
  have_line = true;
  if (text == "S none")
  {
    status = Status();
  }
  else if (text.startsWith("S "))
  {
    status.manager = true;
    status.recording = field(text, "rec") == "1";
    status.duration = field(text, "dur").toFloat();
    status.size = strtoull(field(text, "size").c_str(), nullptr, 10);
    String bags = field(text, "bags");
    status.bags = bags.length() ? bags.toInt() : -1;
    status.phase = field(text, "phase");
    status.target = field(text, "target");
    status.grip = field(text, "grip");
  }
  else if (text.startsWith("R "))
  {
    waiting_reply = false;
    result_ok = text == "R ok";
    result = result_ok ? String("OK") : (text.startsWith("R err ") ? text.substring(6) : text.substring(2));
    result_ms = millis();
  }
  else if (text == "?")
  {
    hello();
  }
}

void readSerial()
{
  while (Serial.available() > 0)
  {
    int c = Serial.read();
    if (c < 0)
      break;
    if (c == '\r')
      continue;
    if (c == '\n')
    {
      line[line_len] = '\0';
      if (!line_overflow)
        handleLine(String(line));
      line_len = 0;
      line_overflow = false;
    }
    else if (line_len < MAX_LINE)
    {
      line[line_len++] = static_cast<char>(c);
    }
    else
    {
      line_overflow = true;   // drop the whole line rather than act on half of it
    }
  }
}

String humanSize(uint64_t bytes)
{
  char text[16];
  if (bytes >= 1000000000ULL)
    snprintf(text, sizeof(text), "%.2f GB", bytes / 1e9);
  else if (bytes >= 1000000ULL)
    snprintf(text, sizeof(text), "%.0f MB", bytes / 1e6);
  else
    snprintf(text, sizeof(text), "%.0f kB", bytes / 1e3);
  return String(text);
}

String clockText(float seconds)
{
  long s = static_cast<long>(seconds);
  char text[16];
  if (s >= 3600)
    snprintf(text, sizeof(text), "%ld:%02ld:%02ld", s / 3600, (s / 60) % 60, s % 60);
  else
    snprintf(text, sizeof(text), "%02ld:%02ld", s / 60, s % 60);
  return String(text);
}

void title(const char* text, uint16_t fg, uint16_t bg, int y)
{
  canvas.setTextColor(fg, bg);
  canvas.setFont(&fonts::FreeSansBold18pt7b);
  canvas.drawString(text, canvas.width() / 2, y);
}

void small(const String& text, uint16_t fg, uint16_t bg, int y)
{
  canvas.setTextColor(fg, bg);
  canvas.setFont(&fonts::FreeSans9pt7b);
  canvas.drawString(text, canvas.width() / 2, y);
}

// a warning strip across the screen
void banner(const char* text, uint16_t fg, uint16_t bg, int y)
{
  canvas.fillRect(0, y - 11, canvas.width(), 22, bg);
  canvas.setTextColor(fg, bg);
  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.drawString(text, canvas.width() / 2, y);
}

void draw(uint32_t now)
{
  bool link = have_line && now - last_line_ms < LINK_TIMEOUT_MS;
  bool blink_on = (now / BLINK_PERIOD_MS) % 2 == 0;
  canvas.setTextDatum(middle_center);
  if (!link)
  {
    canvas.fillScreen(GREY);
    title("NO", WHITE, GREY, 40);
    title("LINK", WHITE, GREY, 76);
    small("start the bridge", WHITE, GREY, 112);
  }
  else if (!status.manager)
  {
    canvas.fillScreen(GREY);
    title("NO", WHITE, GREY, 40);
    title("TELEOP", WHITE, GREY, 76);
    small("no manager status", WHITE, GREY, 112);
  }
  else if (status.recording)
  {
    canvas.fillScreen(RED);
    canvas.fillCircle(18, 22, 9, blink_on ? WHITE : RED);
    title("REC", WHITE, RED, 22);
    small(clockText(status.duration), WHITE, RED, 52);
    small(humanSize(status.size), WHITE, RED, 72);
    bool record_only = status.phase == "record_only";
    if (!record_only && status.phase != "following")
      banner("NO LABEL", BLACK, YELLOW, 96);
    if (!record_only && status.target != "real")
      small("VIRTUAL", YELLOW, RED, 118);
  }
  else
  {
    bool following = status.phase == "following";
    uint16_t bg = following && blink_on ? ORANGE : DARK;
    canvas.fillScreen(bg);
    if (following)
    {
      title("NOT", WHITE, bg, 32);
      title("REC", WHITE, bg, 66);
    }
    else
    {
      title("STAND", WHITE, bg, 32);
      title("BY", WHITE, bg, 66);
    }
    small(status.bags >= 0 ? String(status.bags) + " bags" : String(""), WHITE, bg, 96);
    small(status.phase, WHITE, bg, 116);
  }

  // gripper held by the button (HOLD), top-right corner
  if (status.manager && (status.grip == "open" || status.grip == "closed"))
  {
    bool closed = status.grip == "closed";
    canvas.fillRoundRect(canvas.width() - 44, 2, 42, 16, 4, closed ? YELLOW : WHITE);
    canvas.setTextColor(BLACK, closed ? YELLOW : WHITE);
    canvas.setFont(&fonts::Font0);
    canvas.drawString(closed ? "CLOSE" : "OPEN", canvas.width() - 23, 10);
  }

  if (waiting_reply)
  {
    banner("...", BLACK, WHITE, 64);
  }
  else if (result.length() && now - result_ms < RESULT_SHOW_MS)
  {
    if (result_ok)
    {
      banner("OK", BLACK, WHITE, 64);
    }
    else
    {
      canvas.fillRect(0, 40, canvas.width(), 48, BLACK);
      canvas.setTextColor(YELLOW, BLACK);
      canvas.setFont(&fonts::Font0);   // small enough for the reason
      canvas.setTextDatum(top_left);
      canvas.setCursor(2, 44);
      canvas.setTextWrap(true);
      canvas.print("ERR ");
      canvas.print(result);
      canvas.setTextDatum(middle_center);
    }
  }
  canvas.pushSprite(0, 0);
}
}  // namespace

void setup()
{
  auto config = M5.config();
  config.serial_baudrate = 115200;
  M5.begin(config);
  Serial.setTxTimeoutMs(0);   // never stall the loop when nothing on the PC reads
  M5.Display.setRotation(0);
  M5.BtnA.setHoldThresh(HOLD_MS);
  canvas.createSprite(M5.Display.width(), M5.Display.height());
  hello();
}

void loop()
{
  M5.update();
  readSerial();
  uint32_t now = millis();
  if (waiting_reply && now - click_ms > REPLY_TIMEOUT_MS)
  {
    waiting_reply = false;
    result_ok = false;
    result = "no reply from the PC";
    result_ms = now;
  }
  // one request at a time: presses are ignored until the PC answers. A short
  // press is reported on release (wasClicked), a long one once it passes
  // HOLD_MS (wasHold), so a long press never also sends CLICK.
  if (!waiting_reply && (M5.BtnA.wasClicked() || M5.BtnA.wasHold()))
  {
    Serial.print(M5.BtnA.wasHold() ? "HOLD\n" : "CLICK\n");
    waiting_reply = true;
    click_ms = now;
  }
  static uint32_t last_frame = 0;
  if (now - last_frame >= FRAME_PERIOD_MS)
  {
    last_frame = now;
    draw(now);
  }
  delay(5);
}
