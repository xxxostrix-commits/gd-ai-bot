// =============================================================================
//  GD AI Autopilot - Geode C++ bridge + visualisation engine
//
//  Streams a compact JSON description of the level to a local Python brain over
//  UDP (non-blocking), receives the decided input ("hold") together with the
//  predicted trajectories and renders them with CCDrawNodes inside the level's
//  object layer (world coordinates) plus a light HUD inside PlayLayer.
//
//  Keybinds:
//    F6 - toggle the bot (turning it off instantly releases the button)
//    F7 - cycle visual preset: off -> minimal -> showcase -> debug
//    F8 - cycle behaviour profile: smooth <-> tryhard
//
//  Windows only: uses winsock2 (ws2_32).
// =============================================================================

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>

#include <Geode/Geode.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <Geode/modify/PlayLayer.hpp>
#include <Geode/modify/CCKeyboardDispatcher.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace geode::prelude;

namespace gdai {

// ============================================================================
//  Constants
// ============================================================================

constexpr int kMaxPointsPerPath = 300;
constexpr int kMaxHazardObjects = 200;
constexpr int kMaxAltPaths = 3;
constexpr size_t kSendBufSize = 16384;
constexpr size_t kRecvBufSize = 16384;

// Wire type codes shared with ai-brain/bot.py.
constexpr int kTypeHazard = 1;
constexpr int kTypeSolid = 2;
constexpr int kTypeSlope = 3;
constexpr int kTypeOrb = 4;
constexpr int kTypePad = 5;
constexpr int kTypePortalGravity = 6;
constexpr int kTypePortalMode = 7;
constexpr int kTypePortalSpeed = 8;

// Virtual-key codes for the hotkeys (used through the keyboard dispatcher).
constexpr int kKeyF6 = 0x75;
constexpr int kKeyF7 = 0x76;
constexpr int kKeyF8 = 0x77;

// ============================================================================
//  Compact JSON writer (no external dependencies)
// ============================================================================

class Writer {
public:
    std::string buf;

    void reserve(size_t n) { buf.reserve(n); }
    void ch(char c) { buf.push_back(c); }
    void raw(std::string_view s) { buf.append(s.data(), s.size()); }

    void lit(std::string_view s) {
        buf.push_back('"');
        for (char c : s) {
            switch (c) {
                case '"': buf += "\\\""; break;
                case '\\': buf += "\\\\"; break;
                case '\n': buf += "\\n"; break;
                case '\r': buf += "\\r"; break;
                case '\t': buf += "\\t"; break;
                default:
                    if (static_cast<unsigned char>(c) < 0x20) {
                        buf += "?";
                    } else {
                        buf.push_back(c);
                    }
            }
        }
        buf.push_back('"');
    }

    void key(std::string_view k) {
        lit(k);
        buf.push_back(':');
    }

    void num(double v, int decimals = 2) {
        char tmp[48];
        std::snprintf(tmp, sizeof(tmp), "%.*f", decimals, v);
        size_t n = std::strlen(tmp);
        while (n > 1 && tmp[n - 1] == '0') --n;
        if (n > 1 && tmp[n - 1] == '.') --n;
        if (n == 1 && (tmp[0] == '-' || tmp[0] == '0')) {
            buf.push_back('0');
        } else {
            buf.append(tmp, n);
        }
    }

    void integer(long long v) {
        char tmp[32];
        std::snprintf(tmp, sizeof(tmp), "%lld", v);
        raw(tmp);
    }

    void boolean(bool v) { raw(v ? "true" : "false"); }

    void kv(std::string_view k, double v, int decimals = 2) { key(k); num(v, decimals); }
    void kv(std::string_view k, long long v) { key(k); integer(v); }
    void kv(std::string_view k, bool v) { key(k); boolean(v); }
    void kvs(std::string_view k, std::string_view v) { key(k); lit(v); }
};
// ============================================================================
//  Minimal JSON reader (answers are produced by our own Python brain)
// ============================================================================

inline bool skipWs(const char*& p) {
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') ++p;
    return *p != '\0';
}

inline const char* valueAfter(const std::string& s, const char* key) {
    const char* k = std::strstr(s.c_str(), key);
    if (!k) return nullptr;
    const char* c = k + std::strlen(key);
    skipWs(c);
    if (*c != ':') return nullptr;
    ++c;
    skipWs(c);
    return c;
}

bool readBool(const std::string& s, const char* key, bool& out) {
    const char* v = valueAfter(s, key);
    if (!v) return false;
    if (std::strncmp(v, "true", 4) == 0) { out = true; return true; }
    if (std::strncmp(v, "false", 5) == 0) { out = false; return true; }
    return false;
}

bool readDouble(const std::string& s, const char* key, double& out) {
    const char* v = valueAfter(s, key);
    if (!v || *v == '"') return false;
    char* end = nullptr;
    double d = std::strtod(v, &end);
    if (end == v) return false;
    out = d;
    return true;
}

bool readString(const std::string& s, const char* key, std::string& out) {
    const char* v = valueAfter(s, key);
    if (!v || *v != '"') return false;
    ++v;
    std::string r;
    while (*v && *v != '"') {
        if (*v == '\\' && v[1]) {
            ++v;
            switch (*v) {
                case 'n': r.push_back('\n'); break;
                case 'r': r.push_back('\r'); break;
                case 't': r.push_back('\t'); break;
                default: r.push_back(*v); break;
            }
            ++v;
        } else {
            r.push_back(*v++);
        }
    }
    out = std::move(r);
    return true;
}

// Reads a flat JSON number array: [0,12,55] -> vector<double>.
void readNumberArray(const char* p, std::vector<double>& out) {
    out.clear();
    if (!p || *p != '[') return;
    ++p;
    for (int guard = 0; guard < 20000; ++guard) {
        skipWs(p);
        if (!*p || *p == ']') break;
        char* end = nullptr;
        double d = std::strtod(p, &end);
        if (end == p) break;
        p = end;
        out.push_back(d);
    }
}

// Reads [[x,y],[x,y],...] -> vector<CCPoint>.
void readPointArray(const char* p, std::vector<CCPoint>& out) {
    out.clear();
    if (!p || *p != '[') return;
    ++p;
    for (int guard = 0; guard < 20000; ++guard) {
        skipWs(p);
        if (!*p || *p == ']') break;
        if (*p != '[') { ++p; continue; }
        const char* groupStart = p;
        ++p;
        skipWs(p);
        char* end = nullptr;
        double x = std::strtod(p, &end);
        if (end == p) {
            p = groupStart + 1;
            int depth = 1;
            while (*p && depth > 0) {
                if (*p == '[') ++depth;
                else if (*p == ']') --depth;
                ++p;
            }
            continue;
        }
        p = end;
        skipWs(p);
        if (*p == ',') ++p;
        skipWs(p);
        double y = 0.0;
        if (double d = std::strtod(p, &end); end != p) {
            y = d;
            p = end;
        }
        while (*p && *p != ']') ++p;
        if (*p == ']') ++p;
        out.emplace_back(static_cast<float>(x), static_cast<float>(y));
        if (out.size() >= static_cast<size_t>(kMaxPointsPerPath)) break;
    }
}

// Reads [[[x,y],...],[[x,y],...]] -> vector of point arrays.
void readNestedPaths(const char* p, std::vector<std::vector<CCPoint>>& out) {
    out.clear();
    if (!p || *p != '[') return;
    ++p;
    while (*p && out.size() < static_cast<size_t>(kMaxAltPaths)) {
        skipWs(p);
        if (!*p || *p == ']') break;
        if (*p != '[') { ++p; continue; }
        const char* start = p;
        int depth = 0;
        const char* q = p;
        while (*q) {
            if (*q == '[') ++depth;
            else if (*q == ']') {
                --depth;
                if (depth == 0) { ++q; break; }
            }
            ++q;
        }
        std::string chunk(start, q);
        std::vector<CCPoint> pts;
        readPointArray(chunk.c_str(), pts);
        if (pts.size() >= 2) out.push_back(std::move(pts));
        p = q;
        if (*p == ',') ++p;
    }
// ============================================================================
//  Answer received from the Python brain
// ============================================================================

struct Answer {
    bool valid = false;
    bool hold = false;
    bool hasHold = false;
    double corridorHeight = 0.0;
    std::string hud;

    std::vector<CCPoint> path;
    std::vector<std::vector<CCPoint>> altPaths;
    std::vector<CCPoint> corridorUpper;
    std::vector<CCPoint> corridorLower;
    std::vector<double> risk;

    // Debug snapshot of the scanned objects: 4 numbers + type per object.
    std::vector<double> hazardData;
};

// ============================================================================
//  Visual presets and colours
// ============================================================================

enum class Preset { Off = 0, Minimal = 1, Showcase = 2, Debug = 3 };

const char* presetName(Preset p) {
    switch (p) {
        case Preset::Off: return "off";
        case Preset::Minimal: return "minimal";
        case Preset::Showcase: return "showcase";
        case Preset::Debug: return "debug";
    }
    return "off";
}

Preset presetFromString(std::string_view s) {
    if (s == "minimal") return Preset::Minimal;
    if (s == "showcase") return Preset::Showcase;
    if (s == "debug") return Preset::Debug;
    return Preset::Off;
}

Preset nextPreset(Preset p) {
    switch (p) {
        case Preset::Off: return Preset::Minimal;
        case Preset::Minimal: return Preset::Showcase;
        case Preset::Showcase: return Preset::Debug;
        case Preset::Debug: return Preset::Off;
    }
    return Preset::Off;
}

ccColor4F col4(int r, int g, int b, int a) {
    return ccColor4F{
        static_cast<float>(r) / 255.f,
        static_cast<float>(g) / 255.f,
        static_cast<float>(b) / 255.f,
        static_cast<float>(a) / 255.f,
    };
}

// Neon gradient: cyan -> green -> yellow -> red as `danger` goes 0..100.
ccColor4F dangerColor(double danger, int alpha) {
    const ccColor4F cyan = col4(0, 229, 255, alpha);
    const ccColor4F green = col4(87, 255, 141, alpha);
    const ccColor4F yellow = col4(255, 224, 64, alpha);
    const ccColor4F red = col4(255, 69, 69, alpha);
    double d = std::clamp(danger, 0.0, 100.0);
    auto mix = [](const ccColor4F& a, const ccColor4F& b, float t) {
        return ccColor4F{
            a.r + (b.r - a.r) * t,
            a.g + (b.g - a.g) * t,
            a.b + (b.b - a.b) * t,
            a.a + (b.a - a.a) * t,
        };
    };
    if (d < 35.0) return mix(cyan, green, static_cast<float>(d / 35.0));
    if (d < 70.0) return mix(green, yellow, static_cast<float>((d - 35.0) / 35.0));
    return mix(yellow, red, static_cast<float>((d - 70.0) / 30.0));
}
}
// ============================================================================
//  UDP transport: non-blocking loopback socket, 16 KiB rx/tx buffers
// ============================================================================

class Bridge {
public:
    using Clock = std::chrono::steady_clock;

    bool init(int port) {
        WSADATA data{};
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
            log::error("[gd-ai-bot] WSAStartup failed ({})", WSAGetLastError());
            return false;
        }
        m_wsaStarted = true;

        m_socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (m_socket == INVALID_SOCKET) {
            log::error("[gd-ai-bot] socket() failed ({})", WSAGetLastError());
            return false;
        }

        int buf = static_cast<int>(kSendBufSize);
        ::setsockopt(m_socket, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char*>(&buf), sizeof(buf));
        ::setsockopt(m_socket, SOL_SOCKET, SO_SNDBUF, reinterpret_cast<const char*>(&buf), sizeof(buf));

        u_long nonBlocking = 1;
        if (::ioctlsocket(m_socket, FIONBIO, &nonBlocking) != 0) {
            log::warn("[gd-ai-bot] ioctlsocket(FIONBIO) failed ({})", WSAGetLastError());
        }

        m_addr = sockaddr_in{};
        m_addr.sin_family = AF_INET;
        m_addr.sin_port = htons(static_cast<unsigned short>(port));
        m_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

        m_sendBuffer.assign(kSendBufSize, '\0');
        m_recvBuffer.assign(kRecvBufSize, '\0');

        log::info("[gd-ai-bot] UDP bridge ready on 127.0.0.1:{}", port);
        return true;
    }

    void shutdown() {
        if (m_socket != INVALID_SOCKET) {
            ::closesocket(m_socket);
            m_socket = INVALID_SOCKET;
        }
        if (m_wsaStarted) {
            ::WSACleanup();
            m_wsaStarted = false;
        }
    }

    bool ready() const { return m_socket != INVALID_SOCKET; }
    bool peerSeen() const { return m_peerSeen; }
    double latencyMs() const { return m_latencyMs; }
    long long sequence() const { return m_sequence; }

    static long long nowMs() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   Clock::now().time_since_epoch()
        ).count();
    }

    // Sends a payload built as: {"seq":N,"ts":..., ...}
    bool send(const std::string& payload) {
        if (!ready()) return false;
        size_t len = payload.size();
        if (len + 2 > m_sendBuffer.size()) return false;

        std::memcpy(m_sendBuffer.data(), payload.data(), len);

        lastSend = Clock::now();
        int sent = ::sendto(
            m_socket, m_sendBuffer.data(), static_cast<int>(len), 0,
            reinterpret_cast<const sockaddr*>(&m_addr), sizeof(m_addr)
        );
        if (sent != static_cast<int>(len)) return false;
        m_packetsSent++;
        m_sequence++;
        return true;
    }

    // Drains every pending datagram and keeps the newest answer.
    bool poll(Answer& answer) {
        if (!ready()) return false;
        bool got = false;
        for (int i = 0; i < 64; ++i) {
            sockaddr_in from{};
            int fromLen = sizeof(from);
            int r = ::recvfrom(
                m_socket, m_recvBuffer.data(), static_cast<int>(m_recvBuffer.size()), 0,
                reinterpret_cast<sockaddr*>(&from), &fromLen
            );
            if (r <= 0) break;
            m_peerSeen = true;
            m_packetsReceived++;
            std::string text(m_recvBuffer.data(), static_cast<size_t>(r));
            parseAnswer(text, answer);
            answer.valid = true;
            got = true;
            m_latencyMs = std::chrono::duration<double>(Clock::now() - lastSend).count() * 1000.0;
        }
        return got;
    }

private:
    void parseAnswer(const std::string& s, Answer& a) {
        a.hasHold = readBool(s, "\"hold\"", a.hold);
        readDouble(s, "\"corridor_height\"", a.corridorHeight);
        readString(s, "\"hud\"", a.hud);

        a.path.clear();
        a.altPaths.clear();
        a.corridorUpper.clear();
        a.corridorLower.clear();
        a.risk.clear();
        a.hazardData.clear();

        if (const char* p = valueAfter(s, "\"path\"")) readPointArray(p, a.path);
        if (const char* p = valueAfter(s, "\"alt_paths\"")) readNestedPaths(p, a.altPaths);
        if (const char* p = valueAfter(s, "\"corridor\"")) {
            std::vector<double> flat;
            readNumberArray(p, flat);
            // Flat [x, yTop, yBottom] triples.
            for (size_t i = 0; i + 2 < flat.size(); i += 3) {
                a.corridorUpper.emplace_back(static_cast<float>(flat[i]), static_cast<float>(flat[i + 1]));
                a.corridorLower.emplace_back(static_cast<float>(flat[i]), static_cast<float>(flat[i + 2]));
            }
            if (a.corridorUpper.size() < 2) {
                a.corridorUpper.clear();
                a.corridorLower.clear();
            }
        }
        if (const char* p = valueAfter(s, "\"risk\"")) readNumberArray(p, a.risk);
        if (const char* p = valueAfter(s, "\"hazards\"")) readNumberArray(p, a.hazardData);
    }

    SOCKET m_socket = INVALID_SOCKET;
    bool m_wsaStarted = false;
    bool m_peerSeen = false;
    std::vector<char> m_sendBuffer;
    std::vector<char> m_recvBuffer;
    long long m_sequence = 0;
    unsigned long long m_packetsSent = 0;
    unsigned long long m_packetsReceived = 0;
    double m_latencyMs = 0.0;
    Clock::time_point lastSend = Clock::now();
};
// ============================================================================
//  Global bot state
// ============================================================================

// Compact scanned-object cache, also used by the debug X-ray renderer.
struct ScanObject {
    float x, y, w, h;
    int code;
};

struct BotState {
    Bridge bridge;
    bool bridgeAttempted = false;
    bool bridgeOk = false;
    int bridgePort = -1;
    long long seq = 0;

    bool active = false;   // master switch (F6)
    bool hold = false;     // last value pushed into the game

    Preset preset = Preset::Showcase;
    std::string profile = "smooth";

    Answer answer;
    Ref<CCScene> scene;
    Ref<CCDrawNode> world;   // lives inside m_objectLayer (world space)
    Ref<CCDrawNode> glow;    // same, drawn under `world`
    Ref<CCDrawNode> hud;     // lives inside PlayLayer (UI space)
    Ref<CCLabelBMFont> hudLabel;
    Ref<CCDrawNode> toastPlate;
    Ref<CCLabelBMFont> toastLabel;
    std::chrono::steady_clock::time_point toastUntil{};

    std::chrono::steady_clock::time_point lastAnswer{};
    std::chrono::steady_clock::time_point lastTick{};
    std::vector<ScanObject> scan;   // last scanned hitboxes (debug X-ray)
    std::string sendBuffer;         // reused packet buffer (no per-tick alloc)
};

BotState& state() {
    static BotState s;
    return s;
}

// ============================================================================
//  Object classification for the wire protocol
// ============================================================================

int classifyObject(GameObject* obj) {
    if (!obj) return 0;
    GameObjectType t = obj->m_objectType;
    switch (t) {
        case GameObjectType::Hazard:
        case GameObjectType::AnimatedHazard:
            return kTypeHazard;
        case GameObjectType::Solid:
        case GameObjectType::Breakable:
            return kTypeSolid;
        case GameObjectType::Slope:
            return kTypeSlope;

        case GameObjectType::YellowJumpRing:
        case GameObjectType::PinkJumpRing:
        case GameObjectType::RedJumpRing:
        case GameObjectType::DropRing:
        case GameObjectType::GreenRing:
        case GameObjectType::GravityRing:
        case GameObjectType::CustomRing:
        case GameObjectType::SpiderOrb:
            return kTypeOrb;

        case GameObjectType::YellowJumpPad:
        case GameObjectType::PinkJumpPad:
        case GameObjectType::RedJumpPad:
        case GameObjectType::GravityPad:
        case GameObjectType::SpiderPad:
            return kTypePad;

        case GameObjectType::InverseGravityPortal:
        case GameObjectType::NormalGravityPortal:
        case GameObjectType::GravityTogglePortal:
            return kTypePortalGravity;

        case GameObjectType::CubePortal:
        case GameObjectType::ShipPortal:
        case GameObjectType::BallPortal:
        case GameObjectType::UfoPortal:
        case GameObjectType::WavePortal:
        case GameObjectType::RobotPortal:
        case GameObjectType::SpiderPortal:
        case GameObjectType::SwingPortal:
            return kTypePortalMode;

        case GameObjectType::Decoration:
        case GameObjectType::EnterEffectObject:
        case GameObjectType::Modifier:
        case GameObjectType::Collectible:
        case GameObjectType::SecretCoin:
        case GameObjectType::UserCoin:
            return 0;
        default:
            break;
    }
    // Speed portals: IDs 200..203 are 0.5x..3x, 1331 is 4x.
    if (obj->m_objectID >= 200 && obj->m_objectID <= 203) return kTypePortalSpeed;
    if (obj->m_objectID == 1331) return kTypePortalSpeed;
    return 0;
}

const char* modeName(PlayerObject* p) {
    if (!p) return "cube";
    if (p->m_isShip) return "ship";
    if (p->m_isBall) return "ball";
    if (p->m_isBird) return "ufo";
    if (p->m_isDart) return "wave";
    if (p->m_isRobot) return "robot";
    if (p->m_isSpider) return "spider";
    if (p->m_isSwing) return "swing";
    return "cube";
}

// 10..14 = 0.5x, 1x, 2x, 3x, 4x (same codes as the speed portals).
int speedTier(PlayerObject* p) {
    float s = p ? p->m_playerSpeed : 1.f;
    if (s < 0.8f) return 10;
    if (s < 1.0f) return 11;
    if (s < 1.2f) return 12;
    if (s < 1.45f) return 13;
    return 14;
}

// ============================================================================
//  State packet (C++ -> Python)
// ============================================================================

std::string buildStatePacket(GJBaseGameLayer* self, PlayerObject* player, float dt, long long seq) {
    BotState& st = state();

    const float px = player ? player->getPositionX() : 0.f;
    const float py = player ? player->getPositionY() : 0.f;

    Writer w;
    w.reserve(kSendBufSize - 128);
    w.ch('{');

    w.key("seq");
    w.integer(seq);
    w.ch(',');
    w.kv("ts", Bridge::nowMs());
    w.ch(',');

    // ---- player ----
    w.kvs("mode", modeName(player));
    w.ch(',');
    w.kv("x", static_cast<double>(px), 1);
    w.ch(',');
    w.kv("y", static_cast<double>(py), 1);
    w.ch(',');
    // m_yVelocity is per 60 Hz frame; the brain works in units/second.
    w.kv("vy", player ? static_cast<double>(player->m_yVelocity) * 60.0 : 0.0, 1);
    w.ch(',');
    w.kv("upside_down", player ? player->m_isUpsideDown : false);
    w.ch(',');
    w.kv("mini", player ? (player->m_vehicleSize < 0.8f) : false);
    w.ch(',');
    // "speed" is the raw m_playerSpeed value requested by the spec; the brain
    // prefers "speed_units" (units/second) when it is present.
    w.kv("speed", player ? static_cast<double>(player->m_playerSpeed) : 1.0, 3);
    w.ch(',');
    w.kv("speed_units", player ? static_cast<double>(player->m_playerSpeed) * 311.58 : 311.58, 1);
    w.ch(',');
    w.kv("speed_tier", static_cast<long long>(speedTier(player)));
    w.ch(',');
    w.kv("on_ground", player ? player->m_isOnGround : false);
    w.ch(',');
    w.kv("dead", player ? player->m_isDead : true);
    w.ch(',');

    // ---- flags ----
    w.kvs("visual_preset", presetName(st.preset));
    w.ch(',');
    w.kvs("style_profile", st.profile);
    w.ch(',');
    w.kv("enabled", st.active);
    w.ch(',');
    w.kv("hold", st.hold);
    w.ch(',');
    w.kv("dt", static_cast<double>(dt), 5);
    w.ch(',');
    w.key("ground");
    w.num(0.0);
    w.ch(',');

    // ---- objects in the scan window (-40 .. +520 units around the player) ----
    st.scan.clear();
    w.key("objects");
    w.ch('[');
    bool first = true;
    int emitted = 0;

    if (self && self->m_objects) {
        auto arr = self->m_objects;
        const unsigned int count = arr->count();
        for (unsigned int i = 0; i < count; ++i) {
            auto obj = static_cast<GameObject*>(arr->objectAtIndex(i));
            if (!obj) continue;
            if (obj->m_objectType == GameObjectType::Decoration) continue;
            if (obj->m_isDecoration || obj->m_isDecoration2) continue;
            if (obj->m_isTrigger) continue;

            const float ox = obj->getPositionX();
            if (ox < px - 40.f || ox > px + 520.f) continue;

            const int code = classifyObject(obj);
            if (code == 0) continue;

            auto const& rect = obj->getObjectRect();
            const float rw = rect.size.width;
            const float rh = rect.size.height;
            if (rw < 1.f || rh < 1.f) continue;

            if (!first) w.ch(',');
            first = false;

            w.ch('[');
            w.num(static_cast<double>(rect.origin.x), 1);
            w.ch(',');
            w.num(static_cast<double>(rect.origin.y), 1);
            w.ch(',');
            w.num(static_cast<double>(rw), 1);
            w.ch(',');
            w.num(static_cast<double>(rh), 1);
            w.ch(',');
            w.integer(code);
            w.ch(']');

            st.scan.push_back(ScanObject{ rect.origin.x, rect.origin.y, rw, rh, code });
            if (++emitted >= 900) break;
        }
    }
    w.ch(']');
    w.ch('}');

    return std::move(w.buf);
}
// ============================================================================
//  Drawing helpers (world-space CCDrawNodes)
// ============================================================================

void drawPolyline(CCDrawNode* node, const std::vector<CCPoint>& pts, float width, const ccColor4F& color) {
    if (!node || pts.size() < 2) return;
    for (size_t i = 1; i < pts.size(); ++i) {
        node->drawSegment(pts[i - 1], pts[i], width, color);
    }
}

// Filled rectangle + outline built from primitives (no drawRect dependency).
void drawBox(CCDrawNode* node, const CCRect& r, const ccColor4F& fill, const ccColor4F& border) {
    if (!node) return;
    CCPoint quad[4] = {
        CCPoint(r.origin.x, r.origin.y),
        CCPoint(r.origin.x + r.size.width, r.origin.y),
        CCPoint(r.origin.x + r.size.width, r.origin.y + r.size.height),
        CCPoint(r.origin.x, r.origin.y + r.size.height),
    };
    const ccColor4F none{ 0.f, 0.f, 0.f, 0.f };
    node->drawPolygon(quad, 4, fill, 0.f, none);
    for (int i = 0; i < 4; ++i) {
        node->drawSegment(quad[i], quad[(i + 1) % 4], 1.f, border);
    }
}

// Neon line: wide translucent glow + bright core, coloured by the risk value of
// every point (cyan/green far from obstacles -> yellow/red when close).
void drawNeonPath(CCDrawNode* glow, CCDrawNode* core, const std::vector<CCPoint>& pts,
                  const std::vector<double>& risk, float coreWidth) {
    if (pts.size() < 2) return;
    for (size_t i = 1; i < pts.size(); ++i) {
        const double danger = (i < risk.size()) ? risk[i] : 0.0;
        if (glow) {
            glow->drawSegment(pts[i - 1], pts[i], coreWidth * 4.5f, dangerColor(danger, 30));
        }
        if (core) {
            core->drawSegment(pts[i - 1], pts[i], coreWidth, dangerColor(danger, 230));
        }
    }
}

void drawMarker(CCDrawNode* node, const CCPoint& p, float radius, const ccColor4F& color) {
    if (!node) return;
    node->drawDot(p, radius + 2.f, ccColor4F{ color.r, color.g, color.b, color.a * 0.25f });
    node->drawDot(p, radius, color);
}
// ============================================================================
//  Node lifecycle
// ============================================================================

void ensureNodes(BotState& st) {
    if (st.preset == Preset::Off) return;

    auto play = PlayLayer::get();
    if (!play) return;

    auto scene = CCDirector::sharedDirector()->getRunningScene();
    if (st.scene != scene) {
        // Level ended / scene changed: old nodes died with their parents.
        st.scene = scene;
        st.world = nullptr;
        st.glow = nullptr;
        st.hud = nullptr;
        st.hudLabel = nullptr;
        st.toastPlate = nullptr;
    }

    CCNode* layer = play->m_objectLayer ? static_cast<CCNode*>(play->m_objectLayer) : static_cast<CCNode*>(play);

    if (!st.glow) {
        st.glow = CCDrawNode::create();
        if (st.glow) layer->addChild(st.glow, 9998);
    }
    if (!st.world) {
        st.world = CCDrawNode::create();
        if (st.world) layer->addChild(st.world, 9999);
    }
    if (!st.hud) {
        st.hud = CCDrawNode::create();
        if (st.hud) play->addChild(st.hud, 99999);
    }
    if (!st.hudLabel) {
        st.hudLabel = CCLabelBMFont::create("", "chatFont.fnt");
        if (st.hudLabel) {
            st.hudLabel->setAnchorPoint(CCPoint(0.f, 1.f));
            st.hudLabel->setScale(0.6f);
            st.hudLabel->setOpacity(static_cast<GLubyte>(230));
            play->addChild(st.hudLabel, 100000);
        }
    }
}

void clearOverlay(BotState& st) {
    if (st.world) {
        st.world->clear();
        st.world->setVisible(false);
    }
    if (st.glow) {
        st.glow->clear();
        st.glow->setVisible(false);
    }
    if (st.hud) {
        st.hud->clear();
        st.hud->setVisible(false);
    }
    if (st.hudLabel) {
        st.hudLabel->setString("");
        st.hudLabel->setVisible(false);
    }
    if (st.toastPlate) {
        st.toastPlate->removeFromParentAndCleanup(true);
        st.toastPlate = nullptr;
    }
}
// ============================================================================
//  HUD content + world rendering
// ============================================================================

std::string hudText(const BotState& st, const Answer& a) {
    char state[400];
    if (!st.active) {
        std::snprintf(state, sizeof(state), "OFF (F6 to enable)");
    } else if (!st.bridgeOk) {
        std::snprintf(state, sizeof(state), "socket error - check udp-port");
    } else if (!st.bridge.peerSeen()) {
        std::snprintf(state, sizeof(state), "waiting for ai-brain/bot.py ...");
    } else if (a.hud.empty()) {
        std::snprintf(state, sizeof(state), "connected - computing ...");
    } else {
        std::snprintf(state, sizeof(state), "%s", a.hud.c_str());
    }

    char buf[900];
    if (st.preset == Preset::Debug) {
        std::snprintf(
            buf, sizeof(buf),
            "[debug] %s | %s | corridor %.0f | udp %.1fms | seq %lld | objs %u | %s",
            presetName(st.preset), st.profile.c_str(), a.corridorHeight,
            st.bridge.latencyMs(), st.bridge.sequence(),
            static_cast<unsigned>(st.scan.size()), state
        );
    } else {
        std::snprintf(
            buf, sizeof(buf), "[%s] %s | corridor %.0f | %s",
            presetName(st.preset), st.profile.c_str(), a.corridorHeight, state
        );
    }
    return buf;
}

void renderWorld(BotState& st) {
    if (!st.world) return;
    const Answer& a = st.answer;

    st.world->clear();
    if (st.glow) st.glow->clear();
    st.world->setVisible(true);
    if (st.glow) st.glow->setVisible(true);

    // 1) Main trajectory: thin smooth line (minimal) or neon (showcase/debug).
    if (a.path.size() >= 2) {
        if (st.preset == Preset::Minimal) {
            drawNeonPath(nullptr, st.world, a.path, a.risk, 1.5f);
        } else {
            drawNeonPath(st.glow, st.world, a.path, a.risk, 2.8f);
        }
    }

    if (st.preset == Preset::Showcase || st.preset == Preset::Debug) {
        // 2) Rejected / dangerous alternative branches.
        for (auto const& alt : a.altPaths) {
            drawPolyline(st.world, alt, 1.3f, col4(255, 120, 190, 70));
        }
        // 3) Safety corridor.
        if (a.corridorUpper.size() >= 2 && a.corridorLower.size() == a.corridorUpper.size()) {
            for (size_t i = 1; i < a.corridorUpper.size(); ++i) {
                CCPoint quad[4] = {
                    a.corridorUpper[i - 1], a.corridorUpper[i],
                    a.corridorLower[i], a.corridorLower[i - 1],
                };
                st.world->drawPolygon(quad, 4, col4(110, 255, 210, 24), 0.f, ccColor4F{ 0.f, 0.f, 0.f, 0.f });
                st.world->drawSegment(a.corridorUpper[i - 1], a.corridorUpper[i], 1.f, col4(120, 255, 210, 60));
                st.world->drawSegment(a.corridorLower[i - 1], a.corridorLower[i], 1.f, col4(120, 255, 210, 60));
            }
        }
        // 4) Click markers (next planned click + end of the plan).
        if (a.path.size() >= 2) {
            drawMarker(st.world, a.path.front(), 4.5f, col4(255, 255, 255, 220));
            drawMarker(st.world, a.path.back(), 3.5f, col4(160, 220, 255, 180));
        }
    }

    if (st.preset == Preset::Debug) {
        // 5) X-ray of every scanned hitbox.
        for (auto const& o : st.scan) {
            ccColor4F c = col4(255, 255, 255, 60);
            switch (o.code) {
                case kTypeHazard: c = col4(255, 60, 60, 150); break;
                case kTypeSolid: c = col4(120, 200, 255, 90); break;
                case kTypeSlope: c = col4(80, 160, 255, 90); break;
                case kTypeOrb:
                case kTypePad: c = col4(255, 230, 90, 160); break;
                case kTypePortalGravity:
                case kTypePortalMode:
                case kTypePortalSpeed: c = col4(200, 120, 255, 160); break;
                default: break;
            }
            drawBox(st.world, CCRect(o.x, o.y, o.w, o.h), c, c);
        }

        auto play = PlayLayer::get();
        if (play && play->m_player1) {
            auto p = play->m_player1;
            // 6) The player's own hitbox.
            const float half = 15.f * (p->m_vehicleSize > 0.f ? p->m_vehicleSize : 1.f);
            drawBox(
                st.world,
                CCRect(p->getPositionX() - half, p->getPositionY() - half, half * 2.f, half * 2.f),
                col4(255, 255, 255, 0),
                col4(255, 255, 255, 230)
            );
            // 7) Scan window (-40 .. +520 around the player).
            const float px = p->getPositionX();
            st.world->drawSegment(CCPoint(px - 40.f, -4000.f), CCPoint(px - 40.f, 4000.f), 0.7f, col4(255, 120, 255, 45));
            st.world->drawSegment(CCPoint(px + 520.f, -4000.f), CCPoint(px + 520.f, 4000.f), 0.7f, col4(255, 120, 255, 45));
        }
    }
}

void renderHud(BotState& st) {
    if (!st.hud || !st.hudLabel) return;

    const bool show = st.preset == Preset::Showcase || st.preset == Preset::Debug;
    st.hud->setVisible(show);
    st.hudLabel->setVisible(show);
    if (!show) return;

    st.hudLabel->setString(hudText(st, st.answer).c_str());

    auto win = CCDirector::sharedDirector()->getWinSize();
    const float scale = st.hudLabel->getScale();
    const float labelH = st.hudLabel->getContentSize().height * scale;
    const float labelW = std::min(st.hudLabel->getContentSize().width * scale, win.width - 40.f);
    const float pad = 7.f;
    const float w = labelW + pad * 2.f;
    const float h = labelH + pad * 2.f;
    const CCPoint origin(12.f, win.height - h - 12.f);

    st.hud->clear();
    drawBox(st.hud, CCRect(origin.x, origin.y, w, h), col4(8, 12, 22, 140), col4(70, 240, 220, 70));
    st.hudLabel->setPosition(CCPoint(origin.x + pad, origin.y + h - pad));
}
// ============================================================================
//  Toast: short status popup (never shown in the "off" preset)
// ============================================================================

void showToast(BotState& st, const char* text) {
    if (st.preset == Preset::Off) return;
    auto play = PlayLayer::get();
    if (!play) return;

    if (st.toastLabel) {
        st.toastLabel->removeFromParentAndCleanup(true);
        st.toastLabel = nullptr;
    }
    if (st.toastPlate) {
        st.toastPlate->removeFromParentAndCleanup(true);
        st.toastPlate = nullptr;
    }

    auto size = CCDirector::sharedDirector()->getWinSize();
    auto label = CCLabelBMFont::create(text, "chatFont.fnt");
    if (!label) return;
    label->setScale(0.75f);
    const float lw = label->getContentSize().width * 0.75f;
    const float lh = label->getContentSize().height * 0.75f;

    auto plate = CCDrawNode::create();
    const CCPoint origin(size.width * 0.5f - (lw + 28.f) * 0.5f, size.height * 0.86f);
    if (plate) {
        drawBox(plate, CCRect(origin.x, origin.y, lw + 28.f, lh + 12.f), col4(8, 12, 22, 190), col4(70, 240, 220, 120));
        play->addChild(plate, 999998);
    }
    label->setPosition(CCPoint(size.width * 0.5f, size.height * 0.86f + lh * 0.5f + 5.f));
    play->addChild(label, 999999);

    st.toastPlate = plate;
    st.toastLabel = label;
    st.toastUntil = std::chrono::steady_clock::now() + std::chrono::milliseconds(1400);
}

void expireToast(BotState& st) {
    if (!st.toastLabel) return;
    if (std::chrono::steady_clock::now() < st.toastUntil) return;
    st.toastLabel->removeFromParentAndCleanup(true);
    st.toastLabel = nullptr;
    if (st.toastPlate) {
        st.toastPlate->removeFromParentAndCleanup(true);
        st.toastPlate = nullptr;
    }
}

// ============================================================================
//  Bot control (shared by the hooks and the keybinds)
// ============================================================================

void pushHold(GJBaseGameLayer* self, bool value) {
    BotState& st = state();
    if (st.hold == value) return;
    st.hold = value;
    if (self) {
        self->handleButton(value, 1, true);
    }
}

void setBotActive(GJBaseGameLayer* self, bool value) {
    BotState& st = state();
    st.active = value;
    // Keep the Geode setting in sync so the in-game menu shows the same state.
    Mod::get()->setSettingValue<bool>("bot-enabled", value);
    if (!value) {
        // Turning the bot off: release the button immediately and wipe the
        // overlay so the screen is 100% vanilla again.
        st.hold = false;
        if (self) {
            self->handleButton(false, 1, true);
        }
        st.answer = Answer{};
        clearOverlay(st);
    }
    showToast(st, value ? "AI bot: ON" : "AI bot: OFF");
    log::info("[gd-ai-bot] bot {}", value ? "enabled" : "disabled");
}

void setPreset(GJBaseGameLayer* self, Preset preset, bool showPopup = true) {
    BotState& st = state();
    (void)self;
    st.preset = preset;
    Mod::get()->setSettingValue<std::string>(
        "visual-preset", std::string(presetName(preset)));
    if (preset == Preset::Off) {
        st.answer = Answer{};
        clearOverlay(st);
        return;
    }
    if (showPopup) {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "Visual: %s", presetName(preset));
        showToast(st, buf);
    }
}

void setProfile(const std::string& profile, bool showPopup = true) {
    BotState& st = state();
    st.profile = profile;
    if (showPopup) {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "Profile: %s", profile.c_str());
        showToast(st, buf);
    }
    log::info("[gd-ai-bot] profile set to {}", st.profile);
}

// Called from F6/F7/F8. Without an active level we still update the state so
// the next level starts with the chosen settings.
void handleHotkey(int vk) {
    BotState& st = state();
    GJBaseGameLayer* game = PlayLayer::get();

    switch (vk) {
        case kKeyF6:
            setBotActive(game, !st.active);
            break;
        case kKeyF7:
            setPreset(game, nextPreset(st.preset));
            break;
        case kKeyF8:
            setProfile(st.profile == "smooth" ? "tryhard" : "smooth");
            break;
        default:
            break;
    }
}

} // namespace gdai

using namespace gdai;
// ============================================================================
//  Settings sync + main tick
// ============================================================================

namespace gdai {

void syncSettings(BotState& st) {
    auto mod = Mod::get();

    const bool enabled = mod->getSettingValue<bool>("bot-enabled");
    if (enabled != st.active) {
        st.active = enabled;
        if (!enabled) {
            if (auto play = PlayLayer::get()) {
                st.hold = false;
                play->handleButton(false, 1, true);
            }
            st.answer = Answer{};
            clearOverlay(st);
        }
    }

    const std::string preset = mod->getSettingValue<std::string>("visual-preset");
    const Preset p = presetFromString(preset);
    if (p != st.preset) {
        st.preset = p;
        if (p == Preset::Off) {
            st.answer = Answer{};
            clearOverlay(st);
        }
    }
}

void onLevelReset(PlayLayer* play) {
    BotState& st = state();
    st.hold = false;   // the game already cleared the input state
    st.answer = Answer{};
    st.scan.clear();
    if (st.world) st.world->clear();
    if (st.glow) st.glow->clear();
    if (st.hud) {
        st.hud->clear();
        st.hud->setVisible(false);
    }
    if (st.hudLabel) st.hudLabel->setVisible(false);
    (void)play;
}

void onLevelEnd() {
    BotState& st = state();
    st.hold = false;
    st.answer = Answer{};
    st.scan.clear();
    clearOverlay(st);
}

void tick(GJBaseGameLayer* self, float dt) {
    BotState& st = state();
    if (!self) return;

    auto play = PlayLayer::get();
    if (!play || static_cast<GJBaseGameLayer*>(play) != self) return;

    // processCommands and update can both run in the same frame; only tick once.
    auto now = std::chrono::steady_clock::now();
    if (now - st.lastTick < std::chrono::microseconds(900)) return;
    st.lastTick = now;

    expireToast(st);
    ensureNodes(st);

    // Read the mod settings a few times per second so the Geode menu and the
    // hotkeys always agree (F6/F7 write the same settings).
    static int syncCounter = 0;
    if (++syncCounter >= 30) {
        syncCounter = 0;
        syncSettings(st);

        const int port = static_cast<int>(Mod::get()->getSettingValue<int64_t>("udp-port"));
        if (st.bridgeAttempted && port != st.bridgePort) {
            st.bridge.shutdown();
            st.bridgeAttempted = false;
            st.bridgeOk = false;
            st.bridgePort = -1;
        }
    }

    // Turning the bot off must release the button right away and wipe the visual.
    if (!st.active) {
        if (st.hold) pushHold(self, false);
        clearOverlay(st);
        return;
    }

    auto player = play->m_player1;
    if (!player || player->m_isDead) {
        if (st.hold) pushHold(self, false);
        st.answer = Answer{};
        if (st.preset != Preset::Off) {
            renderWorld(st);
            renderHud(st);
        }
        return;
    }

    if (!st.bridgeAttempted) {
        st.bridgeAttempted = true;
        st.bridgePort = static_cast<int>(Mod::get()->getSettingValue<int64_t>("udp-port"));
        st.bridgeOk = st.bridge.init(st.bridgePort);
        if (!st.bridgeOk) {
            showToast(st, "AI bridge: socket error");
        }
    }
    if (!st.bridgeOk) return;

    // ---- send the level snapshot ----
    st.sendBuffer = buildStatePacket(self, player, dt, ++st.seq);
    st.bridge.send(st.sendBuffer);

    // ---- non-blocking read of the brain's answer ----
    Answer answer;
    if (st.bridge.poll(answer) && answer.hasHold) {
        st.answer = std::move(answer);
        st.lastAnswer = std::chrono::steady_clock::now();
        pushHold(self, st.answer.hold);
    }

    // Safety net: if the brain stops answering, never leave the button pressed.
    if (st.hold) {
        const double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - st.lastAnswer
        ).count();
        if (age > 0.35) {
            pushHold(self, false);
        }
    }

    // ---- render ----
    if (st.preset == Preset::Off) {
        clearOverlay(st);
    } else {
        renderWorld(st);
        renderHud(st);
    }
}

} // namespace gdai
// ============================================================================
//  Mod entry point
// ============================================================================

$on_mod(Loaded) {
    auto mod = Mod::get();
    gdai::BotState& st = gdai::state();

    st.active = mod->getSettingValue<bool>("bot-enabled");
    st.preset = gdai::presetFromString(mod->getSettingValue<std::string>("visual-preset"));
    st.profile = "smooth";

    log::info(
        "[gd-ai-bot] loaded: bot {} | preset {} | udp port {}",
        st.active ? "enabled" : "disabled",
        gdai::presetName(st.preset),
        static_cast<int>(mod->getSettingValue<int64_t>("udp-port"))
    );
    log::info("[gd-ai-bot] F6 = bot on/off, F7 = visual preset, F8 = smooth/tryhard");
}

// ============================================================================
//  Hooks
// ============================================================================

// ---- keyboard: F6 / F7 / F8 -------------------------------------------------

class $modify(BotKeyboardHook, cocos2d::CCKeyboardDispatcher) {
    bool dispatchKeyboardMSG(cocos2d::enumKeyCodes key, bool down, bool repeat) {
        if (down && !repeat) {
            const int vk = static_cast<int>(key);
            if (vk == gdai::kKeyF6 || vk == gdai::kKeyF7 || vk == gdai::kKeyF8) {
                gdai::handleHotkey(vk);
            }
        }
        return cocos2d::CCKeyboardDispatcher::dispatchKeyboardMSG(key, down, repeat);
    }
};

// ---- gameplay: state streaming, input application, drawing ------------------

class $modify(BotGameLayerHook, GJBaseGameLayer) {
    // Primary tick requested by the design: runs every gameplay frame and is
    // where GD processes queued button commands.
    void processCommands(float dt) {
        gdai::tick(this, dt);
        GJBaseGameLayer::processCommands(dt);
    }

    // Fallback: GJBaseGameLayer::update is virtual and guaranteed to run once
    // per frame, so the bridge keeps ticking even if the game skips
    // processCommands on a given frame. gdai::tick() de-duplicates both calls.
    void update(float dt) {
        GJBaseGameLayer::update(dt);
        gdai::tick(this, dt);
    }
};

// ---- level lifecycle -------------------------------------------------------

class $modify(BotPlayLayerHook, PlayLayer) {
    void resetLevel() {
        gdai::onLevelReset(this);
        PlayLayer::resetLevel();
    }

    void onQuit() {
        gdai::onLevelEnd();
        PlayLayer::onQuit();
    }
};