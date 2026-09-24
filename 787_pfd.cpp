#include <XPLMDataAccess.h>
#include <XPLMDisplay.h>
#include <XPLMGraphics.h>
#include <XPLMMenus.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMScenery.h>
#include <XPLMUtilities.h>

#if IBM
#include <windows.h>
#include <GL/gl.h>
#elif APL
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include "nanovg.h"
#define NANOVG_GL2 1
#include "nanovg_gl.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <utility>

namespace {

constexpr int kCanvasWidth = 1200;
constexpr int kCanvasHeight = 900;
constexpr int kWindowWidth = 1000;
constexpr int kWindowHeight = 750;
constexpr int kPfdWindowWidth = 1000;
constexpr int kPfdWindowHeight = 750;
constexpr float kPi = 3.14159265358979323846F;
constexpr float kTerrainDisplayRangeNm = 80.0F;
constexpr int kTerrainRadialSamples = 9;
constexpr int kTerrainBearingSamples = 17;
constexpr int kTerrainSampleCount = kTerrainRadialSamples * kTerrainBearingSamples;
using Color = std::array<float, 3>;

// The earth color is sampled from the dominant flat-ground pixels in the
// supplied 787 PFD reference JPEG (#6E3C07). The image is JPEG-compressed, so
// this is the best reproducible sample from that reference, not an OEM paint spec.
const Color kBlack{0.0F, 0.0F, 0.0F};
const Color kWhite{1.0F, 1.0F, 1.0F};
const Color kMuted{0.60F, 0.68F, 0.76F};
const Color kSky{0.0F, 0.40F, 0.88F};
const Color kEarth{0.43137255F, 0.23529412F, 0.02745098F};
const Color kGreen{0.02F, 0.94F, 0.12F};
const Color kCyan{0.00F, 0.83F, 0.92F};
const Color kMagenta{0.94F, 0.12F, 0.78F};
const Color kAmber{1.00F, 0.69F, 0.00F};
const Color kRed{1.00F, 0.08F, 0.13F};
const Color kDivider{0.24F, 0.29F, 0.34F};
const Color kWindowHeader{0.025F, 0.035F, 0.050F};
const Color kTapeGray{0.34F, 0.37F, 0.40F};

enum class DisplayKind { Pfd, NdEicas, Lower };

struct DisplayWindow {
    DisplayKind kind{};
    const char* title{};
    XPLMWindowID window{};
    int lowerPage{};  // 0 = STATUS, 1 = EFB
    float ndRangeNm{80.0F};
    bool draggingTitle{};
    int dragX{};
    int dragY{};
};

struct DataRefs {
    XPLMDataRef ias{};
    XPLMDataRef altitude{};
    XPLMDataRef verticalSpeed{};
    XPLMDataRef pitch{};
    XPLMDataRef roll{};
    XPLMDataRef heading{};
    XPLMDataRef radioAltitude{};
    XPLMDataRef radioAltitudeDh{};
    XPLMDataRef radioAltitudeDhLit{};
    XPLMDataRef barometer{};
    XPLMDataRef barometerStd{};
    XPLMDataRef barometerWarn{};
    XPLMDataRef gpwsAlert{};
    XPLMDataRef windshearWarning{};
    XPLMDataRef selectedSpeed{};
    XPLMDataRef selectedAltitude{};
    XPLMDataRef selectedHeading{};
    XPLMDataRef flightDirectorMode{};
    XPLMDataRef commandBars{};
    XPLMDataRef flightDirectorPitch{};
    XPLMDataRef flightDirectorRoll{};
    XPLMDataRef flightDirectorMaster{};
    XPLMDataRef autothrottleArm{};
    XPLMDataRef autopilotServos{};
    XPLMDataRef autopilotState{};
    XPLMDataRef gpssStatus{};
    XPLMDataRef fmsVnav{};
    XPLMDataRef vnavStatus{};
    XPLMDataRef navSourceRef{};
    XPLMDataRef ianMode{};
    XPLMDataRef groundSpeed{};
    XPLMDataRef groundTrack{};
    XPLMDataRef latitude{};
    XPLMDataRef longitude{};
    XPLMDataRef elevationMsl{};
    XPLMDataRef terrainEnabled{};
    XPLMDataRef zuluTime{};
    XPLMDataRef flightTime{};
    XPLMDataRef gearHandle{};
    XPLMDataRef flaps{};
    XPLMDataRef speedbrake{};
    XPLMDataRef engineN1{};
    XPLMDataRef engineN2{};
    XPLMDataRef engineEgt{};
    XPLMDataRef engineFuelFlow{};
    XPLMDataRef tcasActiveAdvisory{};
    XPLMDataRef tcasVerticalSpeedBands{};
    XPLMDataRef tcasRelativeBearing{};
    XPLMDataRef tcasRelativeDistance{};
    XPLMDataRef tcasRelativeAltitude{};
    XPLMDataRef tcasTargetThreat{};
    XPLMDataRef viewport{};
} gData;

XPLMMenuID gMenu{};
XPLMFlightLoopID gFlightLoop{};
NVGcontext* gNanoVg{};
int gArialFont{-1};
std::array<DisplayWindow, 3> gDisplays{{
    {DisplayKind::Pfd, "PFD"},
    {DisplayKind::NdEicas, "ND / EICAS SPLIT"},
    {DisplayKind::Lower, "LOWER DISPLAY"}
}};

// Window callbacks are serialized by X-Plane. These values describe the
// currently executing draw callback only.
int gScreenLeft{};
int gScreenTop{};
int gWindowOffsetX{};
int gWindowOffsetY{};
float gScaleX{1.0F};
float gScaleY{1.0F};
float gDevicePixelRatio{1.0F};

struct FmaTransition {
    std::string activeMode;
    float changedAt{};
    bool initialized{};
};

std::array<FmaTransition, 3> gFmaTransitions{};

struct FloatArray {
    std::array<float, 64> values{};
    int count{};
};

struct IntArray {
    std::array<int, 64> values{};
    int count{};
};

struct FlightData {
    float ias{};
    float altitude{};
    float verticalSpeed{};
    float pitch{};
    float roll{};
    float heading{};
    float radioAltitude{};
    float radioAltitudeDh{-1.0F};
    float barometer{};
    int radioAltitudeDhLit{};
    int barometerStd{};
    int barometerWarn{};
    int gpwsAlert{};
    int windshearWarning{};
    float selectedSpeed{-1.0F};
    float selectedAltitude{-1.0F};
    float selectedHeading{-1.0F};
    float flightDirectorPitch{};
    float flightDirectorRoll{};
    int flightDirectorMode{};
    int commandBars{};
    int flightDirectorMaster{};
    int autothrottleArm{};
    int autopilotServos{};
    int autopilotState{};
    int gpssStatus{-1};
    int fmsVnav{};
    int vnavStatus{-1};
    int navSourceRef{-1};
    int ianMode{-1};
    float groundSpeed{};
    float groundTrack{};
    double latitude{};
    double longitude{};
    double elevationMsl{};
    int terrainEnabled{};
    float zuluTime{};
    float flightTime{};
    int gearHandle{};
    float flaps{};
    float speedbrake{};
    FloatArray engineN1{};
    FloatArray engineN2{};
    FloatArray engineEgt{};
    FloatArray engineFuelFlow{};
    int tcasActiveAdvisory{};
    FloatArray tcasVerticalSpeedBands{};
    FloatArray tcasRelativeBearing{};
    FloatArray tcasRelativeDistance{};
    FloatArray tcasRelativeAltitude{};
    IntArray tcasTargetThreat{};
};

FlightData gFlightData{};

struct TerrainSample {
    float distanceNm{};
    float relativeBearing{};
    float elevationFt{};
    bool valid{};
};

std::array<TerrainSample, kTerrainSampleCount> gTerrainSamples{};
XPLMProbeRef gTerrainProbe{};
float gTerrainLastSampleTime{-1.0F};
bool gTerrainSamplesReady{};

float NvgX(float canvasX) {
    return static_cast<float>(gWindowOffsetX) + canvasX * gScaleX;
}

float NvgY(float canvasY) {
    return static_cast<float>(gWindowOffsetY) +
           (static_cast<float>(kCanvasHeight) - canvasY) * gScaleY;
}

NVGcolor NvgColor(const Color& color) {
    return nvgRGBAf(color[0], color[1], color[2], 1.0F);
}

NVGcolor NvgColorAlpha(const Color& color, float alpha) {
    return nvgRGBAf(color[0], color[1], color[2], std::clamp(alpha, 0.0F, 1.0F));
}

void Rect(float left, float bottom, float right, float top, const Color& color) {
    nvgBeginPath(gNanoVg);
    nvgRect(gNanoVg, NvgX(left), NvgY(top), (right - left) * gScaleX,
            (top - bottom) * gScaleY);
    nvgFillColor(gNanoVg, NvgColor(color));
    nvgFill(gNanoVg);
}

void RoundedRect(float left, float bottom, float right, float top, float radius,
                 const Color& color, float alpha = 1.0F) {
    const float scale = (gScaleX + gScaleY) * 0.5F;
    nvgBeginPath(gNanoVg);
    nvgRoundedRect(gNanoVg, NvgX(left), NvgY(top), (right - left) * gScaleX,
                   (top - bottom) * gScaleY, radius * scale);
    nvgFillColor(gNanoVg, NvgColorAlpha(color, alpha));
    nvgFill(gNanoVg);
}

void RoundedRectStroke(float left, float bottom, float right, float top,
                       float radius, const Color& color, float alpha = 1.0F,
                       float width = 1.2F) {
    const float scale = (gScaleX + gScaleY) * 0.5F;
    nvgBeginPath(gNanoVg);
    nvgRoundedRect(gNanoVg, NvgX(left), NvgY(top), (right - left) * gScaleX,
                   (top - bottom) * gScaleY, radius * scale);
    nvgStrokeColor(gNanoVg, NvgColorAlpha(color, alpha));
    nvgStrokeWidth(gNanoVg, std::max(1.0F, width * scale));
    nvgLineJoin(gNanoVg, NVG_ROUND);
    nvgStroke(gNanoVg);
}

bool LoadArialFont() {
#if APL
    constexpr const char* paths[] = {
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/Library/Fonts/Arial.ttf"
    };
#elif IBM
    constexpr const char* paths[] = {
        "C:\\Windows\\Fonts\\arial.ttf",
        "C:\\Windows\\Fonts\\Arial.ttf"
    };
#else
    constexpr const char* paths[] = {
        "/usr/share/fonts/truetype/msttcorefonts/Arial.ttf",
        "/usr/share/fonts/truetype/msttcorefonts/arial.ttf",
        "/usr/share/fonts/truetype/msttcorefonts/Arial.ttf"
    };
#endif
    for (const char* path : paths) {
        std::FILE* file = std::fopen(path, "rb");
        if (!file) continue;
        std::fclose(file);
        gArialFont = nvgCreateFont(gNanoVg, "Arial", path);
        if (gArialFont >= 0) {
            XPLMDebugString("787 Displays: loaded system Arial font for NanoVG text.\n");
            return true;
        }
    }
    XPLMDebugString("787 Displays: Arial.ttf was not found; display labels cannot be rendered.\n");
    return false;
}

void Line(float x1, float y1, float x2, float y2, const Color& color,
          float width = 1.0F) {
    nvgBeginPath(gNanoVg);
    nvgMoveTo(gNanoVg, NvgX(x1), NvgY(y1));
    nvgLineTo(gNanoVg, NvgX(x2), NvgY(y2));
    nvgStrokeColor(gNanoVg, NvgColor(color));
    nvgStrokeWidth(gNanoVg, std::max(1.0F, width * (gScaleX + gScaleY) * 0.5F));
    nvgLineCap(gNanoVg, NVG_ROUND);
    nvgLineJoin(gNanoVg, NVG_ROUND);
    nvgStroke(gNanoVg);
}

void Triangle(float x1, float y1, float x2, float y2, float x3, float y3,
              const Color& color) {
    nvgBeginPath(gNanoVg);
    nvgMoveTo(gNanoVg, NvgX(x1), NvgY(y1));
    nvgLineTo(gNanoVg, NvgX(x2), NvgY(y2));
    nvgLineTo(gNanoVg, NvgX(x3), NvgY(y3));
    nvgClosePath(gNanoVg);
    nvgFillColor(gNanoVg, NvgColor(color));
    nvgFill(gNanoVg);
}

void Circle(float cx, float cy, float radius, const Color& color,
            float width = 1.0F, int segments = 72) {
    (void)segments;
    nvgBeginPath(gNanoVg);
    nvgCircle(gNanoVg, NvgX(cx), NvgY(cy), radius * (gScaleX + gScaleY) * 0.5F);
    nvgStrokeColor(gNanoVg, NvgColor(color));
    nvgStrokeWidth(gNanoVg, std::max(1.0F, width * (gScaleX + gScaleY) * 0.5F));
    nvgStroke(gNanoVg);
}

void Arc(float cx, float cy, float radius, float startDegrees, float endDegrees,
         const Color& color, float width = 1.0F, int segments = 72) {
    nvgBeginPath(gNanoVg);
    for (int i = 0; i <= segments; ++i) {
        const float fraction = static_cast<float>(i) / static_cast<float>(segments);
        const float angle = (startDegrees + (endDegrees - startDegrees) * fraction) *
                            kPi / 180.0F;
        const float x = cx + radius * std::cos(angle);
        const float y = cy + radius * std::sin(angle);
        if (i == 0) nvgMoveTo(gNanoVg, NvgX(x), NvgY(y));
        else nvgLineTo(gNanoVg, NvgX(x), NvgY(y));
    }
    nvgStrokeColor(gNanoVg, NvgColor(color));
    nvgStrokeWidth(gNanoVg, std::max(1.0F, width * (gScaleX + gScaleY) * 0.5F));
    nvgLineCap(gNanoVg, NVG_ROUND);
    nvgLineJoin(gNanoVg, NVG_ROUND);
    nvgStroke(gNanoVg);
}

float TextSize(XPLMFontID font) {
    const float canvasSize = font == xplmFont_Proportional ? 16.0F : 15.0F;
    return canvasSize * (gScaleX + gScaleY) * 0.5F;
}

float MeasureText(const char* value, XPLMFontID font) {
    if (!value || !gNanoVg || gArialFont < 0) return 0.0F;
    nvgSave(gNanoVg);
    nvgFontFaceId(gNanoVg, gArialFont);
    nvgFontSize(gNanoVg, TextSize(font));
    nvgTextAlign(gNanoVg, NVG_ALIGN_LEFT | NVG_ALIGN_BASELINE);
    float bounds[4]{};
    const float width = nvgTextBounds(gNanoVg, 0.0F, 0.0F, value, nullptr, bounds);
    nvgRestore(gNanoVg);
    return width / std::max(0.001F, gScaleX);
}

void Text(float x, float y, const char* value, const Color& color,
          XPLMFontID font = xplmFont_Basic) {
    if (!value || !gNanoVg || gArialFont < 0) return;
    nvgSave(gNanoVg);
    nvgFontFaceId(gNanoVg, gArialFont);
    nvgFontSize(gNanoVg, TextSize(font));
    nvgTextAlign(gNanoVg, NVG_ALIGN_LEFT | NVG_ALIGN_BASELINE);
    nvgFillColor(gNanoVg, NvgColor(color));
    nvgText(gNanoVg, NvgX(x), NvgY(y), value, nullptr);
    nvgRestore(gNanoVg);
}

void CenterText(float cx, float y, const char* value, const Color& color,
                XPLMFontID font = xplmFont_Basic) {
    if (!value) return;
    Text(cx - MeasureText(value, font) * 0.5F, y, value, color, font);
}

float ReadFloat(XPLMDataRef ref, float fallback = 0.0F) {
    return ref ? XPLMGetDataf(ref) : fallback;
}

int ReadInt(XPLMDataRef ref, int fallback = 0) {
    return ref ? XPLMGetDatai(ref) : fallback;
}

double ReadDouble(XPLMDataRef ref, double fallback = 0.0) {
    return ref ? XPLMGetDatad(ref) : fallback;
}

FloatArray ReadFloatArray(XPLMDataRef ref) {
    FloatArray result;
    if (ref) result.count = XPLMGetDatavf(ref, result.values.data(), 0,
                                          static_cast<int>(result.values.size()));
    return result;
}

IntArray ReadIntArray(XPLMDataRef ref) {
    IntArray result;
    if (ref) result.count = XPLMGetDatavi(ref, result.values.data(), 0,
                                          static_cast<int>(result.values.size()));
    return result;
}

FlightData ReadFlightData() {
    FlightData f;
    f.ias = ReadFloat(gData.ias);
    f.altitude = ReadFloat(gData.altitude);
    f.verticalSpeed = ReadFloat(gData.verticalSpeed);
    f.pitch = ReadFloat(gData.pitch);
    f.roll = ReadFloat(gData.roll);
    f.heading = ReadFloat(gData.heading);
    f.radioAltitude = ReadFloat(gData.radioAltitude, -1.0F);
    f.radioAltitudeDh = ReadFloat(gData.radioAltitudeDh, -1.0F);
    f.radioAltitudeDhLit = ReadInt(gData.radioAltitudeDhLit);
    f.barometer = ReadFloat(gData.barometer, -1.0F);
    f.barometerStd = ReadInt(gData.barometerStd);
    f.barometerWarn = ReadInt(gData.barometerWarn);
    f.gpwsAlert = ReadInt(gData.gpwsAlert);
    f.windshearWarning = ReadInt(gData.windshearWarning);
    f.selectedSpeed = ReadFloat(gData.selectedSpeed, -1.0F);
    f.selectedAltitude = ReadFloat(gData.selectedAltitude, -1.0F);
    f.selectedHeading = ReadFloat(gData.selectedHeading, -1.0F);
    f.flightDirectorMode = ReadInt(gData.flightDirectorMode);
    f.commandBars = ReadInt(gData.commandBars, 1);
    f.flightDirectorPitch = ReadFloat(gData.flightDirectorPitch);
    f.flightDirectorRoll = ReadFloat(gData.flightDirectorRoll);
    f.flightDirectorMaster = ReadInt(gData.flightDirectorMaster);
    f.autothrottleArm = ReadInt(gData.autothrottleArm);
    f.autopilotServos = ReadInt(gData.autopilotServos);
    f.autopilotState = ReadInt(gData.autopilotState);
    f.gpssStatus = ReadInt(gData.gpssStatus, -1);
    f.fmsVnav = ReadInt(gData.fmsVnav);
    f.vnavStatus = ReadInt(gData.vnavStatus, -1);
    f.navSourceRef = ReadInt(gData.navSourceRef, -1);
    f.ianMode = ReadInt(gData.ianMode, -1);
    f.groundSpeed = ReadFloat(gData.groundSpeed);
    f.groundTrack = ReadFloat(gData.groundTrack);
    f.latitude = ReadDouble(gData.latitude);
    f.longitude = ReadDouble(gData.longitude);
    f.elevationMsl = ReadDouble(gData.elevationMsl, -1.0);
    f.terrainEnabled = ReadInt(gData.terrainEnabled);
    f.zuluTime = ReadFloat(gData.zuluTime, -1.0F);
    f.flightTime = ReadFloat(gData.flightTime, -1.0F);
    f.gearHandle = ReadInt(gData.gearHandle);
    f.flaps = ReadFloat(gData.flaps);
    f.speedbrake = ReadFloat(gData.speedbrake);
    f.engineN1 = ReadFloatArray(gData.engineN1);
    f.engineN2 = ReadFloatArray(gData.engineN2);
    f.engineEgt = ReadFloatArray(gData.engineEgt);
    f.engineFuelFlow = ReadFloatArray(gData.engineFuelFlow);
    f.tcasActiveAdvisory = ReadInt(gData.tcasActiveAdvisory);
    f.tcasVerticalSpeedBands = ReadFloatArray(gData.tcasVerticalSpeedBands);
    f.tcasRelativeBearing = ReadFloatArray(gData.tcasRelativeBearing);
    f.tcasRelativeDistance = ReadFloatArray(gData.tcasRelativeDistance);
    f.tcasRelativeAltitude = ReadFloatArray(gData.tcasRelativeAltitude);
    f.tcasTargetThreat = ReadIntArray(gData.tcasTargetThreat);
    return f;
}

void UpdateTerrainSamples(const FlightData& f) {
    if (!gTerrainProbe || f.terrainEnabled == 0 ||
        !std::isfinite(f.latitude) || !std::isfinite(f.longitude) ||
        f.latitude < -90.0 || f.latitude > 90.0 || f.longitude < -180.0 ||
        f.longitude > 180.0 || f.elevationMsl < -1000.0) {
        gTerrainSamplesReady = false;
        return;
    }

    const float now = XPLMGetElapsedTime();
    if (gTerrainLastSampleTime >= 0.0F && now - gTerrainLastSampleTime < 1.0F)
        return;
    gTerrainLastSampleTime = now;

    constexpr double kEarthRadiusMeters = 6371008.8;
    constexpr double kDegreesToRadians = static_cast<double>(kPi) / 180.0;
    constexpr double kRadiansToDegrees = 180.0 / static_cast<double>(kPi);
    constexpr double kMetersPerNauticalMile = 1852.0;
    constexpr double kMetersToFeet = 3.280839895;
    const double lat1 = f.latitude * kDegreesToRadians;
    const double lon1 = f.longitude * kDegreesToRadians;
    const double trueHeading = (gData.heading
        ? XPLMDegMagneticToDegTrue(f.heading)
        : XPLMDegMagneticToDegTrue(f.groundTrack)) * kDegreesToRadians;
    int validSamples = 0;

    for (int radial = 0; radial < kTerrainRadialSamples; ++radial) {
        const float distanceNm = kTerrainDisplayRangeNm *
            static_cast<float>(radial + 1) / static_cast<float>(kTerrainRadialSamples);
        const double angularDistance =
            static_cast<double>(distanceNm) * kMetersPerNauticalMile /
            kEarthRadiusMeters;
        for (int bearing = 0; bearing < kTerrainBearingSamples; ++bearing) {
            const int index = radial * kTerrainBearingSamples + bearing;
            auto& sample = gTerrainSamples[static_cast<std::size_t>(index)];
            sample.distanceNm = distanceNm;
            sample.relativeBearing = -90.0F +
                180.0F * static_cast<float>(bearing) /
                static_cast<float>(kTerrainBearingSamples - 1);
            sample.valid = false;

            const double course = trueHeading +
                static_cast<double>(sample.relativeBearing) * kDegreesToRadians;
            const double lat2 = std::asin(std::sin(lat1) * std::cos(angularDistance) +
                std::cos(lat1) * std::sin(angularDistance) * std::cos(course));
            const double lon2 = lon1 + std::atan2(
                std::sin(course) * std::sin(angularDistance) * std::cos(lat1),
                std::cos(angularDistance) - std::sin(lat1) * std::sin(lat2));
            const double latitude = lat2 * kRadiansToDegrees;
            const double longitude = std::remainder(lon2 * kRadiansToDegrees, 360.0);

            double x = 0.0;
            double zeroY = 0.0;
            double z = 0.0;
            XPLMWorldToLocal(latitude, longitude, 0.0, &x, &zeroY, &z);
            XPLMProbeInfo_t info{};
            info.structSize = sizeof(info);
            const XPLMProbeResult result = XPLMProbeTerrainXYZ(
                gTerrainProbe, static_cast<float>(x),
                static_cast<float>(zeroY + 20000.0), static_cast<float>(z), &info);
            if (result != xplm_ProbeHitTerrain) continue;

            sample.elevationFt = static_cast<float>(
                (static_cast<double>(info.locationY) - zeroY) * kMetersToFeet);
            sample.valid = std::isfinite(sample.elevationFt);
            if (sample.valid) ++validSamples;
        }
    }
    gTerrainSamplesReady = validSamples > 0;
}

float UpdateFlightData(float, float, int, void*) {
    // Sample simulator state after the flight model, then let window callbacks
    // render this cached snapshot without polling XPLM from their draw path.
    gFlightData = ReadFlightData();
    UpdateTerrainSamples(gFlightData);
    return 0.05F;
}

void DrawSidebarField(float y, const char* label, const char* value = "---") {
    Text(28.0F, y, label, kCyan);
    Text(94.0F, y, value, kWhite, xplmFont_Proportional);
}

void DrawClockIcon(float cx, float cy) {
    Circle(cx, cy, 39.0F, kMuted, 1.4F, 48);
    for (int minute = 0; minute < 60; minute += 5) {
        const float angle = static_cast<float>(minute) * 6.0F * kPi / 180.0F;
        const float inner = minute % 15 == 0 ? 31.0F : 34.0F;
        Line(cx + inner * std::sin(angle), cy + inner * std::cos(angle),
             cx + 38.0F * std::sin(angle), cy + 38.0F * std::cos(angle),
             kWhite, minute % 15 == 0 ? 1.5F : 0.8F);
    }
    Line(cx, cy, cx - 18.0F, cy + 8.0F, kWhite, 2.0F);
    Line(cx, cy, cx, cy + 25.0F, kWhite, 1.5F);
}

struct AttitudePoint {
    float x{};
    float y{};
};

AttitudePoint TransformAttitudePoint(float x, float y, float cx, float cy,
                                    float cosine, float sine, float pitchOffset) {
    return {cx + x * cosine - y * sine,
            cy + x * sine + y * cosine - pitchOffset};
}

void FillAttitudePolygon(const std::array<AttitudePoint, 4>& points,
                         float cx, float cy, float cosine, float sine,
                         float pitchOffset, const Color& color) {
    const AttitudePoint first = TransformAttitudePoint(points[0].x, points[0].y,
        cx, cy, cosine, sine, pitchOffset);
    nvgBeginPath(gNanoVg);
    nvgMoveTo(gNanoVg, NvgX(first.x), NvgY(first.y));
    for (std::size_t i = 1; i < points.size(); ++i) {
        const AttitudePoint point = TransformAttitudePoint(points[i].x, points[i].y,
            cx, cy, cosine, sine, pitchOffset);
        nvgLineTo(gNanoVg, NvgX(point.x), NvgY(point.y));
    }
    nvgClosePath(gNanoVg);
    nvgFillColor(gNanoVg, NvgColor(color));
    nvgFill(gNanoVg);
}

void DrawAttitudeLine(float x1, float y1, float x2, float y2,
                      float cx, float cy, float cosine, float sine,
                      float pitchOffset, const Color& color, float width) {
    const AttitudePoint first = TransformAttitudePoint(x1, y1, cx, cy,
        cosine, sine, pitchOffset);
    const AttitudePoint second = TransformAttitudePoint(x2, y2, cx, cy,
        cosine, sine, pitchOffset);
    Line(first.x, first.y, second.x, second.y, color, width);
}

void DrawAttitude(const FlightData& f, float clipLeft, float clipRight,
                  float attitudeLeft, float attitudeRight,
                  float bottom, float top) {
    const float cx = (attitudeLeft + attitudeRight) * 0.5F;
    const float cy = (bottom + top) * 0.5F;
    const float rollRadians = std::clamp(f.roll, -180.0F, 180.0F) * kPi / 180.0F;
    const float cosine = std::cos(rollRadians);
    const float sine = std::sin(rollRadians);
    const float pitchOffset = std::clamp(f.pitch, -90.0F, 90.0F) * 8.0F;
    constexpr float extent = 3000.0F;

    // Extend the moving sky/ground field across the tape positions. The tape
    // panels are painted afterward, while this clip keeps the horizon out of
    // the flight-mode area above and compass display below.
    nvgSave(gNanoVg);
    nvgScissor(gNanoVg, NvgX(clipLeft), NvgY(top),
               (clipRight - clipLeft) * gScaleX, (top - bottom) * gScaleY);
    FillAttitudePolygon({AttitudePoint{-extent, 0.0F},
                         AttitudePoint{extent, 0.0F},
                         AttitudePoint{extent, extent},
                         AttitudePoint{-extent, extent}},
                        cx, cy, cosine, sine, pitchOffset, kSky);
    FillAttitudePolygon({AttitudePoint{-extent, -extent},
                         AttitudePoint{extent, -extent},
                         AttitudePoint{extent, 0.0F},
                         AttitudePoint{-extent, 0.0F}},
                        cx, cy, cosine, sine, pitchOffset, kEarth);
    DrawAttitudeLine(-extent, 0.0F, extent, 0.0F, cx, cy,
                     cosine, sine, pitchOffset, kWhite, 2.0F);
    for (int degrees = -30; degrees <= 30; degrees += 5) {
        if (degrees == 0) continue;
        const float y = static_cast<float>(degrees) * 8.0F;
        const float halfWidth = degrees % 10 == 0 ? 68.0F : 35.0F;
        DrawAttitudeLine(-halfWidth, y, halfWidth, y, cx, cy,
                         cosine, sine, pitchOffset, kWhite,
                         degrees % 10 == 0 ? 1.5F : 1.0F);
        if (degrees % 10 == 0) {
            char pitchLabel[8]{};
            std::snprintf(pitchLabel, sizeof(pitchLabel), "%d", std::abs(degrees));
            const AttitudePoint leftLabel = TransformAttitudePoint(
                -halfWidth - 18.0F, y, cx, cy, cosine, sine, pitchOffset);
            const AttitudePoint rightLabel = TransformAttitudePoint(
                halfWidth + 10.0F, y, cx, cy, cosine, sine, pitchOffset);
            Text(leftLabel.x - 18.0F, leftLabel.y - 4.0F, pitchLabel, kWhite);
            Text(rightLabel.x + 4.0F, rightLabel.y - 4.0F, pitchLabel, kWhite);
        }
    }
    nvgRestore(gNanoVg);

    // Fixed bank scale, reference mark, and aircraft symbol.
    const float bankRadius = (top - bottom) * 0.34F;
    const float bankCenterY = top - bankRadius - 12.0F;
    for (int angle = -60; angle <= 60; angle += 10) {
        const float radians = static_cast<float>(angle) * kPi / 180.0F;
        const float inner = bankRadius - (angle % 30 == 0 ? 12.0F : 7.0F);
        const float outer = bankRadius;
        Line(cx + inner * std::sin(radians), bankCenterY + inner * std::cos(radians),
             cx + outer * std::sin(radians), bankCenterY + outer * std::cos(radians),
             kWhite, 1.5F);
    }
    Triangle(cx - 9.0F, bankCenterY + bankRadius - 10.0F,
             cx + 9.0F, bankCenterY + bankRadius - 10.0F,
             cx, bankCenterY + bankRadius + 3.0F, kWhite);
    Line(cx - 55.0F, cy, cx - 20.0F, cy, kWhite, 4.0F);
    Line(cx - 20.0F, cy, cx - 10.0F, cy - 11.0F, kWhite, 4.0F);
    Line(cx - 10.0F, cy - 11.0F, cx + 10.0F, cy - 11.0F, kWhite, 4.0F);
    Line(cx + 10.0F, cy - 11.0F, cx + 20.0F, cy, kWhite, 4.0F);
    Line(cx + 20.0F, cy, cx + 55.0F, cy, kWhite, 4.0F);
    Line(cx, cy - 11.0F, cx, cy - 22.0F, kWhite, 3.0F);

    if (f.flightDirectorMode > 0 && f.commandBars != 0) {
        const float cueX = cx + std::clamp(f.flightDirectorRoll * 3.0F, -100.0F, 100.0F);
        const float cueY = cy + std::clamp(f.flightDirectorPitch * 4.0F, -80.0F, 80.0F);
        // Paired magenta command bars remain visible around the fixed aircraft
        // symbol. Their intersection follows X-Plane's live FD pitch and roll cues.
        Line(cueX - 52.0F, cueY, cueX - 13.0F, cueY, kMagenta, 3.0F);
        Line(cueX + 13.0F, cueY, cueX + 52.0F, cueY, kMagenta, 3.0F);
        Line(cueX, cueY - 20.0F, cueX, cueY - 8.0F, kMagenta, 3.0F);
        Line(cueX, cueY + 8.0F, cueX, cueY + 20.0F, kMagenta, 3.0F);
    }
}

void DrawPfdSpeedTape(const FlightData& f, float left, float right,
                      float bottom, float top) {
    const float center = (bottom + top) * 0.5F;
    RoundedRect(left, bottom, right, top, 8.0F, kTapeGray, 0.70F);
    nvgSave(gNanoVg);
    nvgScissor(gNanoVg, NvgX(left), NvgY(top), (right - left) * gScaleX,
               (top - bottom) * gScaleY);
    const float tapeBottom = bottom + 8.0F;
    const float tapeTop = top - 8.0F;
    const float indicatedAirspeed = std::max(0.0F, f.ias);
    const int first = std::max(0, static_cast<int>(
        std::floor((indicatedAirspeed - 120.0F) / 10.0F)) * 10);
    for (int speed = first; speed <= indicatedAirspeed + 120.0F; speed += 10) {
        const float y = center + (static_cast<float>(speed) - indicatedAirspeed) * 2.0F;
        if (y < tapeBottom || y > tapeTop) continue;
        const bool labeled = speed % 20 == 0;
        Line(right - (labeled ? 22.0F : 12.0F), y, right - 2.0F, y, kWhite,
             labeled ? 1.6F : 1.0F);
        if (labeled) {
            char label[8]{};
            std::snprintf(label, sizeof(label), "%d", speed);
            Text(left + 8.0F, y - 6.0F, label, kWhite, xplmFont_Proportional);
        }
    }
    if (f.selectedSpeed >= 0.0F) {
        const float bugY = center + (f.selectedSpeed - indicatedAirspeed) * 2.0F;
        if (bugY >= tapeBottom && bugY <= tapeTop) {
            Line(right - 29.0F, bugY, right - 7.0F, bugY, kMagenta, 3.0F);
            Triangle(right - 7.0F, bugY, right + 5.0F, bugY + 8.0F,
                     right + 5.0F, bugY - 8.0F, kMagenta);
        }
    }
    nvgRestore(gNanoVg);
    RoundedRect(left, center - 22.0F, right + 4.0F, center + 22.0F,
                4.0F, kBlack, 0.92F);
    RoundedRectStroke(left, center - 22.0F, right + 4.0F, center + 22.0F,
                      4.0F, kWhite, 0.92F, 1.4F);
    Triangle(right + 3.0F, center, right + 15.0F, center + 10.0F,
             right + 15.0F, center - 10.0F, kWhite);
    char value[12]{};
    if (gData.ias) std::snprintf(value, sizeof(value), "%03.0f", std::max(0.0F, f.ias));
    else std::snprintf(value, sizeof(value), "---");
    Text(left + 14.0F, center - 10.0F, value, kWhite, xplmFont_Proportional);
    Text(left + 19.0F, bottom - 25.0F, "KTS", kWhite);
    Text(left + 18.0F, bottom - 49.0F, "IAS", kMuted);
}

void DrawPfdAltitudeTape(const FlightData& f, float left, float right,
                         float bottom, float top) {
    const float center = (bottom + top) * 0.5F;
    RoundedRect(left, bottom, right, top, 8.0F, kTapeGray, 0.70F);
    nvgSave(gNanoVg);
    nvgScissor(gNanoVg, NvgX(left), NvgY(top), (right - left) * gScaleX,
               (top - bottom) * gScaleY);
    const int first = static_cast<int>(std::floor((f.altitude - 1200.0F) / 100.0F)) * 100;
    for (int altitude = first; altitude <= f.altitude + 1200.0F; altitude += 100) {
        const float y = center + (static_cast<float>(altitude) - f.altitude) * 0.20F;
        if (y < bottom + 8.0F || y > top - 8.0F) continue;
        const bool labeled = altitude % 500 == 0;
        Line(left + 2.0F, y, left + (labeled ? 22.0F : 10.0F), y, kWhite,
             labeled ? 1.6F : 1.0F);
        if (labeled) {
            char label[12]{};
            std::snprintf(label, sizeof(label), "%d", altitude);
            Text(left + 25.0F, y - 6.0F, label, kWhite, xplmFont_Proportional);
        }
    }
    if (f.selectedAltitude >= 0.0F) {
        const float bugY = center + (f.selectedAltitude - f.altitude) * 0.20F;
        if (bugY >= bottom + 8.0F && bugY <= top - 8.0F) {
            Line(left + 2.0F, bugY, left + 25.0F, bugY, kMagenta, 3.0F);
            Triangle(left + 2.0F, bugY, left - 9.0F, bugY + 8.0F,
                     left - 9.0F, bugY - 8.0F, kMagenta);
        }
    }
    nvgRestore(gNanoVg);
    RoundedRect(left - 3.0F, center - 22.0F, right + 2.0F, center + 22.0F,
                4.0F, kBlack, 0.92F);
    RoundedRectStroke(left - 3.0F, center - 22.0F, right + 2.0F, center + 22.0F,
                      4.0F, kWhite, 0.92F, 1.4F);
    char value[16]{};
    if (gData.altitude) std::snprintf(value, sizeof(value), "%05.0f", f.altitude);
    else std::snprintf(value, sizeof(value), "---");
    Text(left + 3.0F, center - 10.0F, value, kWhite, xplmFont_Proportional);
    Text(left + 9.0F, bottom - 25.0F, "ALT", kWhite);
    Text(left + 17.0F, bottom - 49.0F, "FT", kMuted);
}

void DrawPfdVerticalSpeed(const FlightData& f, float x, float center,
                          float bottom, float top) {
    const float scaleLimit = std::min(194.0F, std::min(center - bottom - 18.0F,
                                                       top - center - 18.0F));
    const auto drawBand = [&](float low, float high, const Color& color) {
        if (std::abs(high - low) < 1.0F) return;
        const float y1 = center + std::clamp(low * 0.034F, -scaleLimit, scaleLimit);
        const float y2 = center + std::clamp(high * 0.034F, -scaleLimit, scaleLimit);
        RoundedRect(x - 7.0F, std::min(y1, y2), x + 2.0F,
                    std::max(y1, y2), 2.0F, color, 0.55F);
    };
    const bool resolutionAdvisory = f.tcasActiveAdvisory == 2 &&
                                    f.tcasVerticalSpeedBands.count >= 6;
    if (resolutionAdvisory) {
        drawBand(f.tcasVerticalSpeedBands.values[0],
                 f.tcasVerticalSpeedBands.values[1], kRed);
        drawBand(f.tcasVerticalSpeedBands.values[2],
                 f.tcasVerticalSpeedBands.values[3], kGreen);
        drawBand(f.tcasVerticalSpeedBands.values[4],
                 f.tcasVerticalSpeedBands.values[5], kRed);
    }
    Line(x, bottom + 18.0F, x, top - 18.0F, kMuted, 1.0F);
    for (int thousands = -6; thousands <= 6; thousands += 2) {
        const float y = center + static_cast<float>(thousands) * 34.0F;
        Line(x - 10.0F, y, x, y, kWhite, 1.2F);
        if (thousands != 0) {
            char label[8]{};
            std::snprintf(label, sizeof(label), "%d", std::abs(thousands));
            Text(x - 31.0F, y - 5.0F, label, kWhite);
        }
    }
    const float markerY = center + std::clamp(f.verticalSpeed * 0.034F, -194.0F, 194.0F);
    Line(x - 1.0F, center, x - 1.0F, markerY, kGreen, 2.0F);
    Triangle(x - 1.0F, markerY, x - 12.0F, markerY + 8.0F,
             x - 12.0F, markerY - 8.0F, kGreen);
    if (resolutionAdvisory) Text(x - 22.0F, top - 10.0F, "RA", kRed);
    Text(x - 23.0F, bottom - 49.0F, "V/S", kWhite);
}

void TerrainDot(float x, float y, float radius, const Color& color) {
    nvgBeginPath(gNanoVg);
    nvgCircle(gNanoVg, NvgX(x), NvgY(y), radius * (gScaleX + gScaleY) * 0.5F);
    nvgFillColor(gNanoVg, NvgColor(color));
    nvgFill(gNanoVg);
}

void DrawTerrainDots(const FlightData& f, float cx, float cy, float radius,
                     float left, float right, float top) {
    const char* terrainLabel = f.terrainEnabled == 0 ? "TERR OFF" :
        (gTerrainSamplesReady ? "TERR" : "TERR ---");
    const Color& terrainLabelColor = f.terrainEnabled == 0 ? kMuted :
        (gTerrainSamplesReady ? kGreen : kMagenta);
    Text(left + 16.0F, top - 75.0F, terrainLabel, terrainLabelColor,
         xplmFont_Proportional);
    if (f.terrainEnabled == 0) return;
    if (!gTerrainSamplesReady) return;

    float highestTerrain = -std::numeric_limits<float>::infinity();
    float lowestTerrain = std::numeric_limits<float>::infinity();
    for (const auto& sample : gTerrainSamples) {
        if (!sample.valid) continue;
        highestTerrain = std::max(highestTerrain, sample.elevationFt);
        lowestTerrain = std::min(lowestTerrain, sample.elevationFt);
    }
    if (!std::isfinite(highestTerrain) || !std::isfinite(lowestTerrain)) return;

    const float airplaneElevationFt = static_cast<float>(f.elevationMsl * 3.280839895);
    const bool peaksMode = airplaneElevationFt - highestTerrain > 500.0F;
    const float terrainSpan = std::max(1.0F, highestTerrain - lowestTerrain);

    nvgSave(gNanoVg);
    nvgScissor(gNanoVg, NvgX(cx - radius), NvgY(cy + radius),
               radius * 2.0F * gScaleX, radius * gScaleY);
    for (const auto& sample : gTerrainSamples) {
        if (!sample.valid) continue;
        const float radial = sample.distanceNm / kTerrainDisplayRangeNm;
        const float angle = sample.relativeBearing * kPi / 180.0F;
        const float x = cx + radius * radial * std::sin(angle);
        const float y = cy + radius * radial * std::cos(angle);
        const float relativeAltitude = sample.elevationFt - airplaneElevationFt;
        const Color* color = nullptr;
        int density = 0;

        if (peaksMode) {
            // In the 787 terrain display, safely separated terrain is shown
            // as green elevation contours independent of aircraft altitude.
            const float relativePeak =
                (sample.elevationFt - lowestTerrain) / terrainSpan;
            color = &kGreen;
            density = relativePeak >= 0.66F ? 5 : relativePeak >= 0.33F ? 3 : 1;
        } else if (relativeAltitude >= 2000.0F) {
            color = &kRed;
            density = 5;
        } else if (relativeAltitude >= 1000.0F) {
            color = &kAmber;
            density = 5;
        } else if (relativeAltitude >= -500.0F) {
            color = &kAmber;
            density = 3;
        } else if (relativeAltitude >= -1000.0F) {
            color = &kGreen;
            density = 5;
        } else if (relativeAltitude >= -2000.0F) {
            color = &kGreen;
            density = 3;
        }
        if (!color) continue;

        TerrainDot(x, y, 2.0F, *color);
        if (density >= 3) {
            TerrainDot(x - 3.0F, y, 1.5F, *color);
            TerrainDot(x + 3.0F, y, 1.5F, *color);
        }
        if (density >= 5) {
            TerrainDot(x, y - 3.0F, 1.5F, *color);
            TerrainDot(x, y + 3.0F, 1.5F, *color);
        }
    }
    nvgRestore(gNanoVg);
}

void DrawTcasTargets(const FlightData& f, float cx, float cy, float radius) {
    const int count = std::min({f.tcasRelativeBearing.count,
                                f.tcasRelativeDistance.count,
                                f.tcasTargetThreat.count});
    if (count <= 0) return;

    nvgSave(gNanoVg);
    nvgScissor(gNanoVg, NvgX(cx - radius), NvgY(cy + radius),
               radius * 2.0F * gScaleX, radius * gScaleY);
    for (int i = 0; i < count; ++i) {
        const int threat = f.tcasTargetThreat.values[static_cast<std::size_t>(i)];
        if (threat < 1) continue;

        const float bearing = f.tcasRelativeBearing.values[static_cast<std::size_t>(i)];
        const float distanceNm = f.tcasRelativeDistance.values[static_cast<std::size_t>(i)] /
                                 1852.0F;
        if (!std::isfinite(bearing) || !std::isfinite(distanceNm) ||
            std::abs(bearing) > 90.0F || distanceNm < 0.0F ||
            distanceNm > kTerrainDisplayRangeNm) continue;

        const float radial = distanceNm / kTerrainDisplayRangeNm;
        const float angle = bearing * kPi / 180.0F;
        const float x = cx + radius * radial * std::sin(angle);
        const float y = cy + radius * radial * std::cos(angle);
        const Color& color = threat >= 3 ? kRed : threat == 2 ? kAmber : kWhite;

        if (threat >= 3) {
            Rect(x - 5.0F, y - 5.0F, x + 5.0F, y + 5.0F, color);
        } else if (threat == 2) {
            TerrainDot(x, y, 4.5F, color);
        } else {
            Line(x, y + 5.0F, x + 5.0F, y, color, 1.5F);
            Line(x + 5.0F, y, x, y - 5.0F, color, 1.5F);
            Line(x, y - 5.0F, x - 5.0F, y, color, 1.5F);
            Line(x - 5.0F, y, x, y + 5.0F, color, 1.5F);
        }

        if (i < f.tcasRelativeAltitude.count && threat >= 2) {
            const float altitudeHundreds =
                f.tcasRelativeAltitude.values[static_cast<std::size_t>(i)] *
                3.280839895F / 100.0F;
            char altitudeLabel[8]{};
            std::snprintf(altitudeLabel, sizeof(altitudeLabel), "%+03.0f",
                          altitudeHundreds);
            Text(x + 7.0F, y - 6.0F, altitudeLabel, color,
                 xplmFont_Proportional);
        }
    }
    nvgRestore(gNanoVg);
}

struct FmaModes {
    std::array<std::string, 3> active{};  // autothrottle, roll, pitch
    std::array<std::string, 3> armed{};
};

bool HasApMode(const FlightData& f, int flag) {
    return (f.autopilotState & flag) != 0;
}

FmaModes GetFmaModes(const FlightData& f) {
    // X-Plane's documented AP-state bit field is used as the source of truth.
    // It exposes engaged and armed states independently; no modes are commanded here.
    constexpr int kAtSpeed = 1;
    constexpr int kHeadingSelect = 2;
    constexpr int kRollHold = 4;
    constexpr int kSpeedByPitch = 8;
    constexpr int kVerticalSpeed = 16;
    constexpr int kAltitudeArm = 32;
    constexpr int kFlightLevelChange = 64;
    constexpr int kNavArm = 256;
    constexpr int kNavEngaged = 512;
    constexpr int kGlideslopeArm = 1024;
    constexpr int kGlideslopeEngaged = 2048;
    constexpr int kVnavSpeedArm = 4096;
    constexpr int kVnavSpeedEngaged = 8192;
    constexpr int kAltitudeHold = 16384;
    constexpr int kTogaLateral = 32768;
    constexpr int kTogaVertical = 65536;
    constexpr int kVnavPathArm = 131072;
    constexpr int kVnavPathEngaged = 262144;
    constexpr int kGpssEngaged = 524288;
    constexpr int kHeadingHold = 1048576;
    constexpr int kTrackHold = 4194304;
    constexpr int kFlightPathAngle = 8388608;

    FmaModes modes;
    const bool fmsSource = f.navSourceRef == 0 || f.navSourceRef == 1;
    const bool integratedApproach = f.navSourceRef == 1 && f.ianMode == 2;
    const bool locSource = f.navSourceRef == 2 || f.navSourceRef == 3;
    const bool gpssActive = HasApMode(f, kGpssEngaged) || f.gpssStatus == 2;

    if (HasApMode(f, kAtSpeed)) {
        modes.active[0] = "SPD";
    } else if (f.fmsVnav && (HasApMode(f, kFlightLevelChange) ||
                             HasApMode(f, kSpeedByPitch)) &&
               f.autothrottleArm && f.verticalSpeed > 100.0F) {
        // X-Plane's Airliner VNAV climb uses speed-by-pitch with A/T reference thrust.
        modes.active[0] = "THR REF";
    } else if (f.autothrottleArm) {
        modes.armed[0] = "ARM";
    }

    if (HasApMode(f, kTogaLateral)) {
        modes.active[1] = "TO/GA";
    } else if (gpssActive) {
        modes.active[1] = "LNAV";
    } else if (HasApMode(f, kNavEngaged)) {
        if (integratedApproach) modes.active[1] = "FAC";
        else if (locSource) modes.active[1] = "LOC";
        else if (fmsSource) modes.active[1] = "LNAV";
        else modes.active[1] = "NAV";
    } else if (HasApMode(f, kHeadingSelect)) {
        modes.active[1] = "HDG SEL";
    } else if (HasApMode(f, kHeadingHold)) {
        modes.active[1] = "HDG HOLD";
    } else if (HasApMode(f, kTrackHold)) {
        modes.active[1] = "TRK SEL";
    } else if (HasApMode(f, kRollHold)) {
        modes.active[1] = "ATT";
    }

    if (HasApMode(f, kNavArm)) {
        if (integratedApproach) modes.armed[1] = "FAC";
        else if (locSource) modes.armed[1] = "LOC";
        else if (fmsSource || gpssActive) modes.armed[1] = "LNAV";
        else modes.armed[1] = "NAV";
    }

    const bool gpArmed = HasApMode(f, kGlideslopeArm);
    const bool gpCaptured = HasApMode(f, kGlideslopeEngaged);
    const bool vnavPathCaptured = HasApMode(f, kVnavPathEngaged) || f.vnavStatus == 2;
    const bool vnavSpeedCaptured = HasApMode(f, kVnavSpeedEngaged) ||
        (f.fmsVnav && (HasApMode(f, kFlightLevelChange) ||
                       HasApMode(f, kSpeedByPitch)));
    const bool vnavArmed = HasApMode(f, kVnavPathArm) ||
        HasApMode(f, kVnavSpeedArm) || f.vnavStatus == 1;

    if (HasApMode(f, kTogaVertical)) {
        modes.active[2] = "TO/GA";
    } else if (gpCaptured) {
        modes.active[2] = integratedApproach ? "G/P" : "G/S";
    } else if (vnavPathCaptured) {
        modes.active[2] = "VNAV PTH";
    } else if (vnavSpeedCaptured) {
        modes.active[2] = "VNAV SPD";
    } else if (f.fmsVnav && HasApMode(f, kAltitudeHold)) {
        modes.active[2] = "VNAV ALT";
    } else if (HasApMode(f, kFlightLevelChange) || HasApMode(f, kSpeedByPitch)) {
        modes.active[2] = "FLCH SPD";
    } else if (HasApMode(f, kFlightPathAngle)) {
        modes.active[2] = "FPA";
    } else if (HasApMode(f, kVerticalSpeed)) {
        modes.active[2] = "V/S";
    } else if (HasApMode(f, kAltitudeHold)) {
        modes.active[2] = "ALT";
    }

    if (gpArmed) {
        modes.armed[2] = integratedApproach ? "G/P" : "G/S";
    } else if (vnavArmed || (f.fmsVnav && !vnavSpeedCaptured &&
                             !vnavPathCaptured && modes.active[2] != "VNAV ALT")) {
        modes.armed[2] = "VNAV";
    } else if (HasApMode(f, kAltitudeArm)) {
        modes.armed[2] = "ALT";
    }
    return modes;
}

void DrawOutline(float left, float bottom, float right, float top,
                 const Color& color, float width = 1.0F) {
    Line(left, bottom, right, bottom, color, width);
    Line(right, bottom, right, top, color, width);
    Line(right, top, left, top, color, width);
    Line(left, top, left, bottom, color, width);
}

void DrawFma(const FlightData& f, float left, float right, float top) {
    constexpr float kFmaHeight = 80.0F;
    const float bottom = top - kFmaHeight;
    const float col = (right - left) / 3.0F;
    const char* labels[] = {"A/T", "ROLL", "PITCH"};
    const FmaModes modes = GetFmaModes(f);
    const float activeY = top - 37.0F;
    const float armedY = top - 55.0F;
    const float statusY = top - 73.0F;
    const float now = std::max(0.0F, XPLMGetElapsedTime());

    Line(left, bottom, right, bottom, kDivider, 1.0F);
    Line(left, top, right, top, kDivider, 1.0F);
    for (int i = 0; i < 3; ++i) {
        const float x = left + col * static_cast<float>(i);
        if (i > 0) Line(x, bottom, x, top, kDivider, 1.0F);
        const float center = x + col * 0.5F;
        CenterText(center, top - 13.0F, labels[i], kMuted);

        FmaTransition& transition = gFmaTransitions[static_cast<std::size_t>(i)];
        if (!transition.initialized || transition.activeMode != modes.active[i]) {
            transition.activeMode = modes.active[i];
            transition.changedAt = now;
            transition.initialized = true;
        }
        if (!modes.active[i].empty()) {
            CenterText(center, activeY, modes.active[i].c_str(), kGreen,
                       xplmFont_Proportional);
            if (now - transition.changedAt < 10.0F) {
                const float textWidth = MeasureText(modes.active[i].c_str(),
                                                    xplmFont_Proportional);
                DrawOutline(center - textWidth * 0.5F - 7.0F, activeY - 4.0F,
                            center + textWidth * 0.5F + 7.0F, activeY + 15.0F,
                            kGreen, 1.0F);
            }
        }
        if (!modes.armed[i].empty()) {
            CenterText(center, armedY, modes.armed[i].c_str(), kWhite);
        }
    }

    const bool apEngaged = f.autopilotServos > 0 || f.flightDirectorMode == 2;
    if (apEngaged) CenterText((left + right) * 0.5F, statusY, "A/P", kGreen);
    else if (f.flightDirectorMode > 0)
        CenterText((left + right) * 0.5F, statusY, "FLT DIR", kWhite);
}

void DrawPfdCompass(const FlightData& f, float left, float bottom,
                    float right, float top) {
    const float cx = (left + right) * 0.5F;
    const float cy = bottom + 42.0F;
    const float radius = std::min((right - left) * 0.45F, (top - bottom) - 96.0F);
    Line(left, top, right, top, kDivider, 2.0F);
    DrawTerrainDots(f, cx, cy, radius, left, right, top);
    const float heading = std::fmod(f.heading + 360.0F, 360.0F);
    for (int offset = -90; offset <= 90; offset += 5) {
        const float angle = static_cast<float>(offset) * kPi / 180.0F;
        const bool major = offset % 30 == 0;
        const float inner = radius - (major ? 20.0F : 9.0F);
        const float x1 = cx + inner * std::sin(angle);
        const float y1 = cy + inner * std::cos(angle);
        const float x2 = cx + radius * std::sin(angle);
        const float y2 = cy + radius * std::cos(angle);
        Line(x1, y1, x2, y2, kWhite, major ? 1.5F : 0.8F);
        if (major) {
            const int tickHeading = (static_cast<int>(std::lround(heading)) + offset + 360) % 360;
            char label[8]{};
            std::snprintf(label, sizeof(label), "%03d", tickHeading);
            const float labelRadius = radius - 42.0F;
            CenterText(cx + labelRadius * std::sin(angle),
                       cy + labelRadius * std::cos(angle) - 5.0F, label, kWhite);
        }
    }
    DrawTcasTargets(f, cx, cy, radius);
    Arc(cx, cy, radius * 0.62F, 0.0F, 180.0F, kDivider, 1.0F, 64);
    const float selected = f.selectedHeading >= 0.0F ? f.selectedHeading : f.groundTrack;
    float delta = std::fmod(selected - heading + 540.0F, 360.0F) - 180.0F;
    if (std::abs(delta) <= 90.0F) {
        const float angle = delta * kPi / 180.0F;
        const float bx = cx + radius * std::sin(angle);
        const float by = cy + radius * std::cos(angle);
        Line(bx - 8.0F, by - 5.0F, bx, by + 9.0F, kMagenta, 2.0F);
        Line(bx, by + 9.0F, bx + 8.0F, by - 5.0F, kMagenta, 2.0F);
    }
    Line(cx, cy + 28.0F, cx, cy - 10.0F, kWhite, 2.0F);
    Line(cx, cy + 28.0F, cx - 20.0F, cy - 15.0F, kWhite, 2.0F);
    Line(cx, cy + 28.0F, cx + 20.0F, cy - 15.0F, kWhite, 2.0F);
    Triangle(cx - 6.0F, cy + 22.0F, cx + 6.0F, cy + 22.0F,
             cx, cy + 35.0F, kWhite);
    char headingText[32]{};
    if (gData.heading) std::snprintf(headingText, sizeof(headingText), "MAG  %03.0f", heading);
    else std::snprintf(headingText, sizeof(headingText), "MAG  ---");
    Text(cx - 50.0F, bottom + 9.0F, headingText, kWhite, xplmFont_Proportional);

    char groundSpeed[24]{};
    if (gData.groundSpeed)
        std::snprintf(groundSpeed, sizeof(groundSpeed), "GS  %03.0f KT", f.groundSpeed);
    else
        std::snprintf(groundSpeed, sizeof(groundSpeed), "GS  --- KT");
    Text(left + 16.0F, top - 25.0F, groundSpeed, kWhite, xplmFont_Proportional);
    char trackText[24]{};
    if (gData.groundTrack)
        std::snprintf(trackText, sizeof(trackText), "TRK  %03.0f MAG",
                      std::fmod(f.groundTrack + 360.0F, 360.0F));
    else
        std::snprintf(trackText, sizeof(trackText), "TRK  --- MAG");
    CenterText(cx, top - 25.0F, trackText, kGreen, xplmFont_Proportional);
    char rangeText[24]{};
    std::snprintf(rangeText, sizeof(rangeText), "RANGE  %.0f NM",
                  kTerrainDisplayRangeNm);
    Text(right - 137.0F, top - 25.0F, rangeText, kWhite, xplmFont_Proportional);
}

void DrawPfdContent(const FlightData& f) {
    constexpr float sidebarRight = 390.0F;
    constexpr float panelLeft = 402.0F;
    constexpr float panelRight = 1185.0F;
    constexpr float pfdBottom = 286.0F;
    constexpr float pfdTop = 876.0F;
    constexpr float instrumentTop = 782.0F;
    Line(sidebarRight, 18.0F, sidebarRight, 882.0F, kDivider, 2.0F);
    DrawClockIcon(312.0F, 808.0F);
    DrawSidebarField(852.0F, "FLT");
    DrawSidebarField(816.0F, "MIC");
    DrawSidebarField(780.0F, "XPDR");
    DrawSidebarField(744.0F, "SELCAL");
    DrawSidebarField(708.0F, "TAIL");
    Line(24.0F, 678.0F, 370.0F, 678.0F, kDivider, 1.2F);
    char utc[20]{};
    if (gData.zuluTime && f.zuluTime >= 0.0F) {
        const int totalSeconds = static_cast<int>(std::floor(f.zuluTime)) % 86400;
        std::snprintf(utc, sizeof(utc), "%02d:%02d:%02dZ", totalSeconds / 3600,
                      (totalSeconds / 60) % 60, totalSeconds % 60);
    } else {
        std::snprintf(utc, sizeof(utc), "--:--:--Z");
    }
    Text(24.0F, 650.0F, "UTC TIME", kCyan);
    Text(24.0F, 622.0F, utc, kWhite, xplmFont_Proportional);
    Text(146.0F, 650.0F, "DATE", kCyan);
    Text(146.0F, 622.0F, "--- -- --", kWhite, xplmFont_Proportional);
    char elapsed[20]{};
    if (gData.flightTime && f.flightTime >= 0.0F) {
        const int totalMinutes = static_cast<int>(f.flightTime / 60.0F);
        std::snprintf(elapsed, sizeof(elapsed), "%02d:%02d", totalMinutes / 60,
                      totalMinutes % 60);
    } else {
        std::snprintf(elapsed, sizeof(elapsed), "--:--");
    }
    Text(262.0F, 650.0F, "ELAPSED TIME", kCyan);
    Text(262.0F, 622.0F, elapsed, kWhite, xplmFont_Proportional);

    DrawFma(f, panelLeft, panelRight, pfdTop);
    const float attitudeLeft = panelLeft + 104.0F;
    const float attitudeRight = panelRight - 140.0F;
    const float tapeBottom = pfdBottom + 24.0F;
    const float speedLeft = panelLeft + 6.0F;
    const float speedRight = attitudeLeft - 12.0F;
    const float altitudeLeft = attitudeRight + 12.0F;
    const float altitudeRight = panelRight - 70.0F;
    // Draw a full-span, borderless moving horizon first; the translucent tapes
    // then sit above it without interrupting the sky/ground split.
    DrawAttitude(f, speedLeft, panelRight - 6.0F,
                 attitudeLeft, attitudeRight, tapeBottom, instrumentTop);
    DrawPfdSpeedTape(f, speedLeft, speedRight, tapeBottom, instrumentTop);
    DrawPfdAltitudeTape(f, altitudeLeft, altitudeRight,
                        tapeBottom, instrumentTop);
    DrawPfdVerticalSpeed(f, panelRight - 20.0F,
                         (tapeBottom + instrumentTop) * 0.5F,
                         tapeBottom, instrumentTop);

    const char* alertText = nullptr;
    const Color* alertColor = nullptr;
    if (f.gpwsAlert != 0) {
        alertText = "PULL UP";
        alertColor = &kRed;
    } else if (f.windshearWarning >= 1 && f.windshearWarning <= 2) {
        alertText = "WINDSHEAR";
        alertColor = &kAmber;
    } else if (f.windshearWarning >= 3) {
        alertText = "WINDSHEAR";
        alertColor = &kRed;
    }
    if (alertText) {
        const float attitudeCenterX = (attitudeLeft + attitudeRight) * 0.5F;
        const float attitudeCenterY = (tapeBottom + instrumentTop) * 0.5F;
        nvgSave(gNanoVg);
        nvgScissor(gNanoVg, NvgX(speedLeft), NvgY(instrumentTop),
                   (panelRight - 6.0F - speedLeft) * gScaleX,
                   (instrumentTop - tapeBottom) * gScaleY);
        CenterText(attitudeCenterX, attitudeCenterY - 8.0F,
                   alertText, *alertColor, xplmFont_Proportional);
        nvgRestore(gNanoVg);
    }
    DrawPfdCompass(f, panelLeft, 24.0F, panelRight, tapeBottom);

    char ra[32]{};
    if (f.radioAltitude >= 0.0F && f.radioAltitude < 2500.0F)
        std::snprintf(ra, sizeof(ra), "RA  %04.0f", f.radioAltitude);
    else std::snprintf(ra, sizeof(ra), "RA  ---");
    Text(panelLeft + 18.0F, 26.0F, ra,
         f.radioAltitudeDhLit ? kAmber : kGreen);
    if (f.radioAltitudeDh > 0.0F) {
        char dh[24]{};
        std::snprintf(dh, sizeof(dh), "DH  %04.0f", f.radioAltitudeDh);
        Text(panelLeft + 118.0F, 26.0F, dh,
             f.radioAltitudeDhLit ? kAmber : kWhite);
    }

    constexpr float kInHgToHpa = 33.8638866667F;
    const bool pressureAvailable = f.barometerStd != 0 || f.barometer > 0.0F;
    const float pressureHpa = f.barometerStd != 0
        ? 1013.25F : f.barometer * kInHgToHpa;
    char baro[32]{};
    if (pressureAvailable) {
        std::snprintf(baro, sizeof(baro), "%s %04.0f HPA",
                      f.barometerStd ? "STD" : "QNH", pressureHpa);
    } else {
        std::snprintf(baro, sizeof(baro), "%s ---- HPA",
                      f.barometerStd ? "STD" : "QNH");
    }
    const float baroX = panelRight - 180.0F;
    if (f.barometerWarn != 0) {
        const float textWidth = MeasureText(baro, xplmFont_Proportional);
        RoundedRectStroke(baroX - 7.0F, 13.0F, baroX + textWidth + 8.0F,
                          40.0F, 3.0F, kAmber, 1.0F, 1.6F);
    }
    Text(baroX, 26.0F, baro,
         f.barometerWarn ? kAmber : kWhite, xplmFont_Proportional);
}

void DrawNdArc(const FlightData& f, const DisplayWindow& display,
               float left, float right, float bottom, float top) {
    const float cx = (left + right) * 0.5F;
    const float cy = bottom + 124.0F;
    const float radius = std::min((right - left) * 0.45F, top - cy - 42.0F);
    const float heading = std::fmod(f.heading + 360.0F, 360.0F);
    Text(left + 20.0F, top - 35.0F, "MAP", kCyan, xplmFont_Proportional);
    char track[32]{};
    if (gData.heading) std::snprintf(track, sizeof(track), "TRK  %03.0f  MAG", heading);
    else std::snprintf(track, sizeof(track), "TRK  ---  MAG");
    CenterText(cx, top - 35.0F, track, kGreen, xplmFont_Proportional);
    char range[24]{};
    std::snprintf(range, sizeof(range), "RANGE  %.0f NM", display.ndRangeNm);
    Text(left + 20.0F, top - 76.0F, range, kWhite);
    char gs[24]{};
    if (gData.groundSpeed) std::snprintf(gs, sizeof(gs), "GS  %.0f KT", f.groundSpeed);
    else std::snprintf(gs, sizeof(gs), "GS  --- KT");
    Text(left + 20.0F, top - 108.0F, gs, kWhite);

    for (int rangeIndex = 1; rangeIndex <= 2; ++rangeIndex) {
        const float arcRadius = radius * static_cast<float>(rangeIndex) / 2.0F;
        Arc(cx, cy, arcRadius, 0.0F, 180.0F, rangeIndex == 2 ? kMuted : kDivider,
            rangeIndex == 2 ? 1.5F : 1.0F, 64);
        char ringValue[12]{};
        std::snprintf(ringValue, sizeof(ringValue), "%.0f",
                      display.ndRangeNm * static_cast<float>(rangeIndex) / 2.0F);
        Text(cx + 7.0F, cy + arcRadius - 17.0F, ringValue, kMuted);
    }

    for (int relative = -105; relative <= 105; relative += 5) {
        const float angle = static_cast<float>(relative) * kPi / 180.0F;
        const bool major = relative % 30 == 0;
        const bool medium = relative % 10 == 0;
        const float inner = radius - (major ? 22.0F : (medium ? 14.0F : 7.0F));
        const float x1 = cx + inner * std::sin(angle);
        const float y1 = cy + inner * std::cos(angle);
        const float x2 = cx + radius * std::sin(angle);
        const float y2 = cy + radius * std::cos(angle);
        Line(x1, y1, x2, y2, kWhite, major ? 1.7F : 0.9F);
        if (major) {
            const int labelHeading = (static_cast<int>(std::lround(heading)) + relative + 360) % 360;
            char label[8]{};
            std::snprintf(label, sizeof(label), "%03d", labelHeading);
            const float labelRadius = radius - 45.0F;
            CenterText(cx + labelRadius * std::sin(angle),
                       cy + labelRadius * std::cos(angle) - 6.0F, label, kWhite);
        }
    }

    if (f.selectedHeading >= 0.0F) {
        const float delta = std::fmod(f.selectedHeading - heading + 540.0F, 360.0F) - 180.0F;
        if (std::abs(delta) <= 105.0F) {
            const float angle = delta * kPi / 180.0F;
            const float bx = cx + radius * std::sin(angle);
            const float by = cy + radius * std::cos(angle);
            Line(bx - 8.0F, by - 5.0F, bx, by + 8.0F, kMagenta, 2.0F);
            Line(bx, by + 8.0F, bx + 8.0F, by - 5.0F, kMagenta, 2.0F);
        }
    }
    const float trackDelta = std::fmod(f.groundTrack - heading + 540.0F, 360.0F) - 180.0F;
    if (gData.groundTrack && std::abs(trackDelta) <= 105.0F) {
        const float angle = trackDelta * kPi / 180.0F;
        Line(cx, cy + 24.0F, cx + radius * 0.77F * std::sin(angle),
             cy + radius * 0.77F * std::cos(angle), kGreen, 2.0F);
    }
    Line(cx, cy + 30.0F, cx, cy - 8.0F, kWhite, 2.0F);
    Line(cx, cy + 30.0F, cx - 20.0F, cy - 14.0F, kWhite, 2.0F);
    Line(cx, cy + 30.0F, cx + 20.0F, cy - 14.0F, kWhite, 2.0F);
    Triangle(cx - 6.0F, cy + 24.0F, cx + 6.0F, cy + 24.0F,
             cx, cy + 37.0F, kAmber);
    Text(left + 20.0F, bottom + 24.0F, "RANGE: MOUSE WHEEL", kMuted);
    if (gData.latitude && gData.longitude) {
        char pos[48]{};
        std::snprintf(pos, sizeof(pos), "POS  %.3f  %.3f", f.latitude, f.longitude);
        Text(left + 20.0F, bottom + 54.0F, pos, kWhite);
    } else {
        Text(left + 20.0F, bottom + 54.0F, "POS  ---  ---", kWhite);
    }
}

void DrawGauge(float cx, float cy, float radius, const char* label,
               const char* value, float fraction, const Color& valueColor) {
    Text(cx - radius, cy + radius + 26.0F, label, kCyan);
    Arc(cx, cy, radius, 0.0F, 180.0F, kMuted, 1.2F, 42);
    for (int tick = 0; tick <= 10; ++tick) {
        const float angle = static_cast<float>(tick) * 18.0F * kPi / 180.0F;
        const float inner = radius - (tick % 2 == 0 ? 12.0F : 7.0F);
        Line(cx + inner * std::cos(angle), cy + inner * std::sin(angle),
             cx + radius * std::cos(angle), cy + radius * std::sin(angle),
             kWhite, tick % 2 == 0 ? 1.1F : 0.7F);
    }
    const float angle = std::clamp(fraction, 0.0F, 1.0F) * 180.0F * kPi / 180.0F;
    Line(cx, cy, cx + (radius - 13.0F) * std::cos(angle),
         cy + (radius - 13.0F) * std::sin(angle), valueColor, 2.2F);
    Circle(cx, cy, 4.0F, valueColor, 1.0F, 16);
    Rect(cx - 58.0F, cy - 31.0F, cx + 58.0F, cy - 4.0F, kDivider);
    Rect(cx - 56.0F, cy - 29.0F, cx + 56.0F, cy - 6.0F, kBlack);
    CenterText(cx, cy - 25.0F, value, valueColor, xplmFont_Proportional);
}

bool EngineValue(const FloatArray& array, int index, float& value) {
    if (index < 0 || index >= array.count || index >= static_cast<int>(array.values.size()))
        return false;
    value = array.values[static_cast<std::size_t>(index)];
    return true;
}

void DrawEngineGauge(float cx, float cy, const char* label, const FloatArray& data,
                     int engine, float maximum, const char* format,
                     const Color& valueColor, float flowScale = 1.0F) {
    float raw{};
    char value[20]{};
    float fraction = 0.0F;
    if (EngineValue(data, engine, raw)) {
        raw *= flowScale;
        std::snprintf(value, sizeof(value), format, raw);
        fraction = raw / maximum;
    } else {
        std::snprintf(value, sizeof(value), "---");
    }
    DrawGauge(cx, cy, 63.0F, label, value, fraction, valueColor);
}

void DrawEicasPanel(const FlightData& f, float left, float right,
                    float bottom, float top) {
    const float width = right - left;
    const float engine1 = left + width * 0.29F;
    const float engine2 = left + width * 0.73F;
    Text(left + 20.0F, top - 38.0F, "ENGINE INDICATIONS", kCyan,
         xplmFont_Proportional);
    CenterText(engine1, top - 74.0F, "ENGINE 1", kWhite);
    CenterText(engine2, top - 74.0F, "ENGINE 2", kWhite);
    Line((engine1 + engine2) * 0.5F, top - 90.0F,
         (engine1 + engine2) * 0.5F, bottom + 215.0F, kDivider, 1.0F);
    const std::array<float, 4> rowY{{top - 210.0F, top - 355.0F,
                                     top - 500.0F, top - 645.0F}};
    for (float y : rowY) Line(left + 12.0F, y - 52.0F, right - 12.0F, y - 52.0F,
                              kDivider, 0.8F);
    const float maxEngine1 = engine1;
    const float maxEngine2 = engine2;
    DrawEngineGauge(maxEngine1, rowY[0], "N1 %", f.engineN1, 0, 110.0F,
                    "%.1f", kGreen);
    DrawEngineGauge(maxEngine2, rowY[0], "N1 %", f.engineN1, 1, 110.0F,
                    "%.1f", kGreen);
    DrawEngineGauge(maxEngine1, rowY[1], "EGT C", f.engineEgt, 0, 1000.0F,
                    "%.0f", kAmber);
    DrawEngineGauge(maxEngine2, rowY[1], "EGT C", f.engineEgt, 1, 1000.0F,
                    "%.0f", kAmber);
    DrawEngineGauge(maxEngine1, rowY[2], "N2 %", f.engineN2, 0, 110.0F,
                    "%.1f", kGreen);
    DrawEngineGauge(maxEngine2, rowY[2], "N2 %", f.engineN2, 1, 110.0F,
                    "%.1f", kGreen);
    DrawEngineGauge(maxEngine1, rowY[3], "FUEL KG/H", f.engineFuelFlow, 0, 20000.0F,
                    "%.0f", kCyan, 3600.0F);
    DrawEngineGauge(maxEngine2, rowY[3], "FUEL KG/H", f.engineFuelFlow, 1, 20000.0F,
                    "%.0f", kCyan, 3600.0F);

    const float statusTop = bottom + 156.0F;
    Line(left + 12.0F, statusTop, right - 12.0F, statusTop, kDivider, 1.4F);
    Text(left + 18.0F, statusTop - 31.0F, "FLIGHT CONTROL STATUS", kCyan);
    const auto field = [&](float x, const char* label, const char* value,
                          const Color& color) {
        Text(x, statusTop - 62.0F, label, kMuted);
        Text(x, statusTop - 96.0F, value, color, xplmFont_Proportional);
    };
    char gear[24]{};
    if (gData.gearHandle) std::snprintf(gear, sizeof(gear), "%s", f.gearHandle ? "DOWN" : "UP");
    else std::snprintf(gear, sizeof(gear), "---");
    char flap[24]{};
    if (gData.flaps) std::snprintf(flap, sizeof(flap), "%.0f %%", std::clamp(f.flaps, 0.0F, 1.0F) * 100.0F);
    else std::snprintf(flap, sizeof(flap), "---");
    char speedbrake[24]{};
    if (gData.speedbrake) std::snprintf(speedbrake, sizeof(speedbrake), "%.0f %%", std::clamp(f.speedbrake, 0.0F, 1.0F) * 100.0F);
    else std::snprintf(speedbrake, sizeof(speedbrake), "---");
    field(left + 30.0F, "GEAR HANDLE", gear, f.gearHandle ? kGreen : kWhite);
    field(left + width * 0.39F, "FLAP HANDLE", flap, kWhite);
    field(left + width * 0.72F, "SPEEDBRAKE", speedbrake, kWhite);
}

void DrawNdEicasContent(const FlightData& f, const DisplayWindow& display) {
    constexpr float divider = 590.0F;
    Line(divider, 20.0F, divider, 880.0F, kDivider, 2.0F);
    DrawNdArc(f, display, 18.0F, divider - 15.0F, 24.0F, 875.0F);
    DrawEicasPanel(f, divider + 10.0F, 1185.0F, 24.0F, 875.0F);
}

void DrawLowerTabs(const DisplayWindow& display) {
    const bool status = display.lowerPage == 0;
    Rect(850.0F, 24.0F, 1012.0F, 78.0F, status ? kDivider : kBlack);
    Rect(1020.0F, 24.0F, 1174.0F, 78.0F, status ? kBlack : kDivider);
    Text(887.0F, 43.0F, "STATUS", status ? kCyan : kWhite, xplmFont_Proportional);
    Text(1070.0F, 43.0F, "EFB", status ? kWhite : kCyan, xplmFont_Proportional);
}

void DrawLowerContent(const FlightData& f, const DisplayWindow& display) {
    Text(30.0F, 846.0F, "LOWER DISPLAY", kWhite, xplmFont_Proportional);
    Line(28.0F, 816.0F, 1172.0F, 816.0F, kDivider, 1.5F);
    if (display.lowerPage == 0) {
        Text(34.0F, 770.0F, "AIRCRAFT STATUS", kCyan, xplmFont_Proportional);
        const std::array<std::pair<float, float>, 4> cards{{
            {34.0F, 310.0F}, {326.0F, 602.0F}, {618.0F, 894.0F}, {910.0F, 1166.0F}
        }};
        const char* labels[] = {"IAS (KT)", "ALTITUDE (FT)", "GROUND SPEED", "VERTICAL SPEED"};
        char values[4][32]{};
        if (gData.ias) std::snprintf(values[0], sizeof(values[0]), "%.0f", f.ias);
        else std::snprintf(values[0], sizeof(values[0]), "---");
        if (gData.altitude) std::snprintf(values[1], sizeof(values[1]), "%.0f", f.altitude);
        else std::snprintf(values[1], sizeof(values[1]), "---");
        if (gData.groundSpeed) std::snprintf(values[2], sizeof(values[2]), "%.0f KT", f.groundSpeed);
        else std::snprintf(values[2], sizeof(values[2]), "---");
        if (gData.verticalSpeed) std::snprintf(values[3], sizeof(values[3]), "%.0f FPM", f.verticalSpeed);
        else std::snprintf(values[3], sizeof(values[3]), "---");
        for (std::size_t i = 0; i < cards.size(); ++i) {
            Rect(cards[i].first, 560.0F, cards[i].second, 710.0F, kDivider);
            Rect(cards[i].first + 2.0F, 562.0F, cards[i].second - 2.0F, 708.0F, kBlack);
            Text(cards[i].first + 18.0F, 674.0F, labels[i], kMuted);
            Text(cards[i].first + 18.0F, 610.0F, values[i], kWhite,
                 xplmFont_Proportional);
        }
        char gear[32]{};
        if (gData.gearHandle) std::snprintf(gear, sizeof(gear), "GEAR HANDLE  %s", f.gearHandle ? "DOWN" : "UP");
        else std::snprintf(gear, sizeof(gear), "GEAR HANDLE  ---");
        Text(52.0F, 472.0F, gear, f.gearHandle ? kGreen : kWhite);
        char flap[32]{};
        if (gData.flaps) std::snprintf(flap, sizeof(flap), "FLAP HANDLE  %.0f %%", std::clamp(f.flaps, 0.0F, 1.0F) * 100.0F);
        else std::snprintf(flap, sizeof(flap), "FLAP HANDLE  ---");
        Text(390.0F, 472.0F, flap, kWhite);
        char spoiler[32]{};
        if (gData.speedbrake) std::snprintf(spoiler, sizeof(spoiler), "SPEEDBRAKE  %.0f %%", std::clamp(f.speedbrake, 0.0F, 1.0F) * 100.0F);
        else std::snprintf(spoiler, sizeof(spoiler), "SPEEDBRAKE  ---");
        Text(760.0F, 472.0F, spoiler, kWhite);
        if (gData.latitude && gData.longitude) {
            char position[56]{};
            std::snprintf(position, sizeof(position), "POSITION  %.4f  %.4f", f.latitude, f.longitude);
            Text(52.0F, 390.0F, position, kWhite);
        } else {
            Text(52.0F, 390.0F, "POSITION  ---  ---", kWhite);
        }
        Text(52.0F, 344.0F, "STATUS VALUES COME FROM THE ACTIVE X-PLANE AIRCRAFT.", kMuted);
    } else {
        Text(34.0F, 770.0F, "ELECTRONIC FLIGHT BAG  /  OVERVIEW", kCyan,
             xplmFont_Proportional);
        Rect(34.0F, 286.0F, 580.0F, 704.0F, kDivider);
        Rect(36.0F, 288.0F, 578.0F, 702.0F, kBlack);
        Text(62.0F, 658.0F, "FLIGHT OVERVIEW", kWhite);
        char value[64]{};
        if (gData.latitude) std::snprintf(value, sizeof(value), "LAT  %.4f", f.latitude);
        else std::snprintf(value, sizeof(value), "LAT  ---");
        Text(62.0F, 598.0F, value, kWhite);
        if (gData.longitude) std::snprintf(value, sizeof(value), "LON  %.4f", f.longitude);
        else std::snprintf(value, sizeof(value), "LON  ---");
        Text(62.0F, 554.0F, value, kWhite);
        if (gData.altitude) std::snprintf(value, sizeof(value), "ALT  %.0f FT", f.altitude);
        else std::snprintf(value, sizeof(value), "ALT  ---");
        Text(62.0F, 490.0F, value, kWhite);
        if (gData.groundSpeed) std::snprintf(value, sizeof(value), "GS   %.0f KT", f.groundSpeed);
        else std::snprintf(value, sizeof(value), "GS   ---");
        Text(62.0F, 446.0F, value, kWhite);
        Rect(610.0F, 286.0F, 1166.0F, 704.0F, kDivider);
        Rect(612.0F, 288.0F, 1164.0F, 702.0F, kBlack);
        Text(638.0F, 658.0F, "AIRCRAFT STATUS", kWhite);
        char status[48]{};
        if (gData.gearHandle) std::snprintf(status, sizeof(status), "GEAR HANDLE  %s", f.gearHandle ? "DOWN" : "UP");
        else std::snprintf(status, sizeof(status), "GEAR HANDLE  ---");
        Text(638.0F, 598.0F, status, f.gearHandle ? kGreen : kWhite);
        if (gData.flaps) std::snprintf(status, sizeof(status), "FLAP HANDLE  %.0f %%", std::clamp(f.flaps, 0.0F, 1.0F) * 100.0F);
        else std::snprintf(status, sizeof(status), "FLAP HANDLE  ---");
        Text(638.0F, 554.0F, status, kWhite);
        if (gData.speedbrake) std::snprintf(status, sizeof(status), "SPEEDBRAKE  %.0f %%", std::clamp(f.speedbrake, 0.0F, 1.0F) * 100.0F);
        else std::snprintf(status, sizeof(status), "SPEEDBRAKE  ---");
        Text(638.0F, 510.0F, status, kWhite);
        Text(62.0F, 228.0F, "CHARTS, PERFORMANCE AND DOCUMENTS ARE NOT CONNECTED.", kMuted);
    }
    DrawLowerTabs(display);
}

void DrawWindowChrome(const DisplayWindow& display) {
    constexpr float kChromeBottom = 882.0F;
    Rect(0.0F, kChromeBottom, static_cast<float>(kCanvasWidth),
         static_cast<float>(kCanvasHeight), kWindowHeader);
    Line(0.0F, kChromeBottom, static_cast<float>(kCanvasWidth),
         kChromeBottom, kDivider, 1.0F);
    CenterText(static_cast<float>(kCanvasWidth) * 0.5F, 886.0F,
               display.title, kCyan, xplmFont_Proportional);

    Rect(1164.0F, 883.0F, 1197.0F, 899.0F, kDivider);
    Rect(1165.0F, 884.0F, 1196.0F, 898.0F, kWindowHeader);
    Line(1177.0F, 888.0F, 1184.0F, 895.0F, kWhite, 1.5F);
    Line(1184.0F, 888.0F, 1177.0F, 895.0F, kWhite, 1.5F);
}

void RestorePluginOpenGLState() {
    // Restore NanoVG's unmanaged state to X-Plane's documented UI baseline.
    // Managed enables and the 2-D texture binding go through XPLM APIs.
    XPLMSetGraphicsState(0, 0, 0, 0, 0, 0, 0);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBlendEquation(GL_FUNC_ADD);
    glFrontFace(GL_CW);
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
    glDisable(GL_STENCIL_TEST);
    glStencilMask(0xffffffffU);
    glStencilFunc(GL_ALWAYS, 0, 0xffffffffU);
    glStencilOpSeparate(GL_FRONT, GL_KEEP, GL_KEEP, GL_KEEP);
    glStencilOpSeparate(GL_BACK, GL_KEEP, GL_KEEP, GL_KEEP);
    glActiveTexture(GL_TEXTURE0);
    XPLMBindTexture2d(0, 0);
}

void DrawDisplay(XPLMWindowID window, DisplayWindow& display) {
    int left = 0;
    int top = 0;
    int right = kWindowWidth;
    int bottom = 0;
    int screenRight = 0;
    int screenBottom = 0;
    XPLMGetWindowGeometry(window, &left, &top, &right, &bottom);
    XPLMGetScreenBoundsGlobal(&gScreenLeft, &gScreenTop,
                              &screenRight, &screenBottom);
    const int screenWidth = std::max(1, screenRight - gScreenLeft);
    const int screenHeight = std::max(1, gScreenTop - screenBottom);
    const int width = std::max(1, right - left);
    const int height = std::max(1, top - bottom);
    gWindowOffsetX = left - gScreenLeft;
    gWindowOffsetY = gScreenTop - top;
    gScaleX = static_cast<float>(width) / static_cast<float>(kCanvasWidth);
    gScaleY = static_cast<float>(height) / static_cast<float>(kCanvasHeight);

    std::array<int, 4> viewport{};
    const int viewportValues = gData.viewport
        ? XPLMGetDatavi(gData.viewport, viewport.data(), 0,
                        static_cast<int>(viewport.size()))
        : 0;
    if (viewportValues == static_cast<int>(viewport.size()) &&
        viewport[2] > 0 && viewport[3] > 0) {
        const float pixelScaleX = static_cast<float>(viewport[2]) /
                                  static_cast<float>(screenWidth);
        const float pixelScaleY = static_cast<float>(viewport[3]) /
                                  static_cast<float>(screenHeight);
        gDevicePixelRatio = std::clamp(std::max(pixelScaleX, pixelScaleY), 1.0F, 4.0F);
    } else {
        gDevicePixelRatio = 1.0F;
    }

    XPLMSetGraphicsState(0, 0, 0, 0, 0, 0, 0);
    if (!gNanoVg) {
        gNanoVg = nvgCreateGL2(NVG_ANTIALIAS);
        if (!gNanoVg) {
            XPLMDebugString("787 Displays: NanoVG OpenGL 2 context creation failed.\n");
            RestorePluginOpenGLState();
            return;
        }
        LoadArialFont();
    }

    nvgBeginFrame(gNanoVg, static_cast<float>(screenWidth),
                  static_cast<float>(screenHeight), gDevicePixelRatio);
    nvgScissor(gNanoVg, static_cast<float>(gWindowOffsetX),
               static_cast<float>(gWindowOffsetY),
               static_cast<float>(width), static_cast<float>(height));

    // Self-decorated XPLM windows have no simulator-provided backing. Paint the
    // full client area opaque black before any instrument marks are submitted.
    Rect(0.0F, 0.0F, static_cast<float>(kCanvasWidth),
         static_cast<float>(kCanvasHeight), kBlack);

    const FlightData& f = gFlightData;
    switch (display.kind) {
        case DisplayKind::Pfd: DrawPfdContent(f); break;
        case DisplayKind::NdEicas: DrawNdEicasContent(f, display); break;
        case DisplayKind::Lower: DrawLowerContent(f, display); break;
    }
    DrawWindowChrome(display);

    nvgEndFrame(gNanoVg);
    RestorePluginOpenGLState();
}

void DrawWindow(XPLMWindowID window, void* refcon) {
    auto* display = static_cast<DisplayWindow*>(refcon);
    if (display) DrawDisplay(window, *display);
}

int HandleMouseClick(XPLMWindowID window, int x, int y,
                     XPLMMouseStatus status, void* refcon) {
    auto* display = static_cast<DisplayWindow*>(refcon);
    if (!display) return 1;
    int left = 0;
    int top = 0;
    int right = 0;
    int bottom = 0;
    XPLMGetWindowGeometry(window, &left, &top, &right, &bottom);
    const float canvasX = static_cast<float>(x - left) *
                          static_cast<float>(kCanvasWidth) / std::max(1, right - left);
    const float canvasY = static_cast<float>(y - bottom) *
                          static_cast<float>(kCanvasHeight) / std::max(1, top - bottom);

    if (status == xplm_MouseDown && canvasY >= 882.0F) {
        if (canvasX >= 1164.0F) {
            display->draggingTitle = false;
            XPLMSetWindowIsVisible(window, 0);
        } else {
            display->draggingTitle = true;
            display->dragX = x;
            display->dragY = y;
        }
        return 1;
    }
    if (status == xplm_MouseDrag && display->draggingTitle) {
        const int dx = x - display->dragX;
        const int dy = y - display->dragY;
        XPLMSetWindowGeometry(window, left + dx, top + dy,
                              right + dx, bottom + dy);
        display->dragX = x;
        display->dragY = y;
        return 1;
    }
    if (status == xplm_MouseUp) {
        display->draggingTitle = false;
        return 1;
    }
    if (status == xplm_MouseDown && display->kind == DisplayKind::Lower &&
        canvasY >= 24.0F && canvasY <= 78.0F) {
        if (canvasX >= 850.0F && canvasX <= 1012.0F) display->lowerPage = 0;
        if (canvasX >= 1020.0F && canvasX <= 1174.0F) display->lowerPage = 1;
    }
    return 1;
}

void HandleKey(XPLMWindowID, char, XPLMKeyFlags, char, void*, int) {}

XPLMCursorStatus HandleCursor(XPLMWindowID, int, int, void*) {
    return xplm_CursorDefault;
}

int HandleMouseWheel(XPLMWindowID, int, int, int wheel, int clicks, void* refcon) {
    auto* display = static_cast<DisplayWindow*>(refcon);
    if (display && display->kind == DisplayKind::NdEicas && wheel == 0) {
        const float factor = std::pow(2.0F, static_cast<float>(clicks));
        display->ndRangeNm = std::clamp(display->ndRangeNm / factor, 10.0F, 640.0F);
    }
    return 1;
}

void ToggleDisplay(void*, void* itemRef) {
    auto* display = static_cast<DisplayWindow*>(itemRef);
    if (!display || !display->window) return;
    XPLMSetWindowIsVisible(display->window, !XPLMGetWindowIsVisible(display->window));
}

void BindDataRefs() {
    gData.ias = XPLMFindDataRef("sim/cockpit2/gauges/indicators/airspeed_kts_pilot");
    gData.altitude = XPLMFindDataRef("sim/cockpit2/gauges/indicators/altitude_ft_pilot");
    gData.verticalSpeed = XPLMFindDataRef("sim/cockpit2/gauges/indicators/vvi_fpm_pilot");
    gData.pitch = XPLMFindDataRef("sim/cockpit2/gauges/indicators/pitch_AHARS_deg_pilot");
    gData.roll = XPLMFindDataRef("sim/cockpit2/gauges/indicators/roll_AHARS_deg_pilot");
    gData.heading = XPLMFindDataRef("sim/cockpit2/gauges/indicators/heading_AHARS_deg_mag_pilot");
    gData.radioAltitude = XPLMFindDataRef("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot");
    gData.radioAltitudeDh = XPLMFindDataRef("sim/cockpit2/gauges/actuators/radio_altimeter_bug_ft_pilot");
    gData.radioAltitudeDhLit = XPLMFindDataRef("sim/cockpit2/gauges/indicators/radio_altimeter_dh_lit_pilot");
    gData.barometer = XPLMFindDataRef("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot");
    gData.barometerStd = XPLMFindDataRef("sim/cockpit2/gauges/actuators/barometer_setting_is_std_pilot");
    gData.barometerWarn = XPLMFindDataRef("sim/cockpit2/gauges/actuators/barometer_setting_warn_pilot");
    gData.gpwsAlert = XPLMFindDataRef("sim/cockpit2/annunciators/GPWS");
    gData.windshearWarning = XPLMFindDataRef("sim/cockpit2/annunciators/windshear_warning_systems");
    gData.selectedSpeed = XPLMFindDataRef("sim/cockpit2/autopilot/airspeed_dial_kts");
    gData.selectedAltitude = XPLMFindDataRef("sim/cockpit2/autopilot/altitude_dial_ft");
    gData.selectedHeading = XPLMFindDataRef("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot");
    gData.flightDirectorMode = XPLMFindDataRef("sim/cockpit2/autopilot/flight_director_mode");
    gData.commandBars = XPLMFindDataRef("sim/cockpit2/autopilot/flight_director_command_bars_pilot");
    gData.flightDirectorPitch = XPLMFindDataRef("sim/cockpit2/autopilot/flight_director_pitch_deg");
    gData.flightDirectorRoll = XPLMFindDataRef("sim/cockpit2/autopilot/flight_director_roll_deg");
    gData.flightDirectorMaster = XPLMFindDataRef("sim/cockpit2/autopilot/flight_director_master_pilot");
    gData.autothrottleArm = XPLMFindDataRef("sim/cockpit2/autopilot/autothrottle_arm");
    gData.autopilotServos = XPLMFindDataRef("sim/cockpit2/autopilot/servos_on");
    gData.autopilotState = XPLMFindDataRef("sim/cockpit/autopilot/autopilot_state");
    gData.gpssStatus = XPLMFindDataRef("sim/cockpit2/autopilot/gpss_status");
    gData.fmsVnav = XPLMFindDataRef("sim/cockpit2/autopilot/fms_vnav");
    gData.vnavStatus = XPLMFindDataRef("sim/cockpit2/autopilot/vnav_status");
    gData.navSourceRef = XPLMFindDataRef("sim/cockpit2/radios/indicators/nav_src_ref");
    gData.ianMode = XPLMFindDataRef("sim/cockpit2/radios/indicators/ian_mode");
    gData.groundSpeed = XPLMFindDataRef("sim/cockpit2/gauges/indicators/ground_speed_kt");
    gData.groundTrack = XPLMFindDataRef("sim/cockpit2/gauges/indicators/ground_track_mag_pilot");
    gData.latitude = XPLMFindDataRef("sim/flightmodel/position/latitude");
    gData.longitude = XPLMFindDataRef("sim/flightmodel/position/longitude");
    gData.elevationMsl = XPLMFindDataRef("sim/flightmodel/position/elevation");
    gData.terrainEnabled = XPLMFindDataRef("sim/cockpit2/EFIS/EFIS_terrain_on");
    gData.tcasActiveAdvisory = XPLMFindDataRef("sim/cockpit2/tcas/indicators/tcas_active_advisory");
    gData.tcasVerticalSpeedBands = XPLMFindDataRef("sim/cockpit2/tcas/indicators/tcas_vs_bands");
    gData.tcasRelativeBearing = XPLMFindDataRef("sim/cockpit2/tcas/indicators/relative_bearing_degs");
    gData.tcasRelativeDistance = XPLMFindDataRef("sim/cockpit2/tcas/indicators/relative_distance_mtrs");
    gData.tcasRelativeAltitude = XPLMFindDataRef("sim/cockpit2/tcas/indicators/relative_altitude_mtrs");
    gData.tcasTargetThreat = XPLMFindDataRef("sim/cockpit2/tcas/targets/threat");
    gData.zuluTime = XPLMFindDataRef("sim/time/zulu_time_sec");
    gData.flightTime = XPLMFindDataRef("sim/time/total_flight_time_sec");
    gData.gearHandle = XPLMFindDataRef("sim/cockpit2/controls/gear_handle_down");
    gData.flaps = XPLMFindDataRef("sim/cockpit2/controls/flap_handle_request_ratio");
    gData.speedbrake = XPLMFindDataRef("sim/cockpit2/controls/speedbrake_ratio");
    gData.engineN1 = XPLMFindDataRef("sim/cockpit2/engine/indicators/N1_percent");
    gData.engineN2 = XPLMFindDataRef("sim/cockpit2/engine/indicators/N2_percent");
    gData.engineEgt = XPLMFindDataRef("sim/cockpit2/engine/indicators/EGT_deg_C");
    gData.engineFuelFlow = XPLMFindDataRef("sim/cockpit2/engine/indicators/fuel_flow_kg_sec");
    gData.viewport = XPLMFindDataRef("sim/graphics/view/viewport");
}

bool CreateDisplayWindow(DisplayWindow& display, int index) {
    int screenLeft = 0;
    int screenTop = 0;
    int screenRight = 1440;
    int screenBottom = 900;
    XPLMGetScreenBoundsGlobal(&screenLeft, &screenTop, &screenRight, &screenBottom);

    const int width = display.kind == DisplayKind::Pfd
        ? kPfdWindowWidth : kWindowWidth;
    const int height = display.kind == DisplayKind::Pfd
        ? kPfdWindowHeight : kWindowHeight;
    XPLMCreateWindow_t params{};
    params.structSize = sizeof(params);
    params.left = screenLeft + std::max(20, (screenRight - screenLeft - width) / 2) + index * 28;
    params.top = screenTop - std::max(50, (screenTop - screenBottom - height) / 3) - index * 24;
    params.right = params.left + width;
    params.bottom = params.top - height;
    params.visible = 0;
    params.drawWindowFunc = DrawWindow;
    params.handleMouseClickFunc = HandleMouseClick;
    params.handleKeyFunc = HandleKey;
    params.handleCursorFunc = HandleCursor;
    params.handleMouseWheelFunc = HandleMouseWheel;
    params.refcon = &display;
    // No X-Plane frame or backing: DrawDisplay paints the complete black surface
    // and our own header inside the callback.
    params.decorateAsFloatingWindow = xplm_WindowDecorationSelfDecorated;
    params.layer = xplm_WindowLayerFloatingWindows;
    params.handleRightClickFunc = nullptr;

    display.window = XPLMCreateWindowEx(&params);
    if (!display.window) return false;
    XPLMSetWindowResizingLimits(display.window, width, height, width, height);
    return true;
}

void CreatePluginMenu() {
    XPLMMenuID plugins = XPLMFindPluginsMenu();
    const int parentItem = XPLMAppendMenuItem(plugins, "787 Displays", nullptr, 1);
    gMenu = XPLMCreateMenu("787 Displays", plugins, parentItem, ToggleDisplay, nullptr);
    if (!gMenu) return;
    for (auto& display : gDisplays) {
        char label[64]{};
        std::snprintf(label, sizeof(label), "Show / hide %s", display.title);
        XPLMAppendMenuItem(gMenu, label, &display, 1);
    }
}

void DestroyPluginMenu() {
    if (gMenu) {
        XPLMDestroyMenu(gMenu);
        gMenu = nullptr;
    }
}

}  // namespace

PLUGIN_API int XPluginStart(char* outName, char* outSignature, char* outDescription) {
    std::snprintf(outName, 256, "787 Displays");
    std::snprintf(outSignature, 256, "org.787x.displays");
    std::snprintf(outDescription, 256,
                  "PFD, split ND/EICAS, and independently switchable lower display.");
    BindDataRefs();
    gTerrainProbe = XPLMCreateProbe(xplm_ProbeY);
    gFlightData = ReadFlightData();
    UpdateTerrainSamples(gFlightData);
    XPLMCreateFlightLoop_t loopParams{};
    loopParams.structSize = sizeof(loopParams);
    loopParams.phase = xplm_FlightLoop_Phase_AfterFlightModel;
    loopParams.callbackFunc = UpdateFlightData;
    gFlightLoop = XPLMCreateFlightLoop(&loopParams);
    if (!gFlightLoop) {
        if (gTerrainProbe) {
            XPLMDestroyProbe(gTerrainProbe);
            gTerrainProbe = nullptr;
        }
        return 0;
    }
    XPLMScheduleFlightLoop(gFlightLoop, -1.0F, 1);

    for (std::size_t i = 0; i < gDisplays.size(); ++i) {
        if (!CreateDisplayWindow(gDisplays[i], static_cast<int>(i))) {
            for (auto& display : gDisplays) {
                if (display.window) {
                    XPLMDestroyWindow(display.window);
                    display.window = nullptr;
                }
            }
            XPLMDestroyFlightLoop(gFlightLoop);
            gFlightLoop = nullptr;
            if (gTerrainProbe) {
                XPLMDestroyProbe(gTerrainProbe);
                gTerrainProbe = nullptr;
            }
            return 0;
        }
    }
    CreatePluginMenu();
    if (!gMenu) {
        for (auto& display : gDisplays) {
            if (display.window) {
                XPLMDestroyWindow(display.window);
                display.window = nullptr;
            }
        }
        DestroyPluginMenu();
        XPLMDestroyFlightLoop(gFlightLoop);
        gFlightLoop = nullptr;
        if (gTerrainProbe) {
            XPLMDestroyProbe(gTerrainProbe);
            gTerrainProbe = nullptr;
        }
        return 0;
    }
    return 1;
}

PLUGIN_API void XPluginStop(void) {
    DestroyPluginMenu();
    if (gFlightLoop) {
        XPLMDestroyFlightLoop(gFlightLoop);
        gFlightLoop = nullptr;
    }
    if (gTerrainProbe) {
        XPLMDestroyProbe(gTerrainProbe);
        gTerrainProbe = nullptr;
    }
    for (auto& display : gDisplays) {
        if (display.window) {
            XPLMDestroyWindow(display.window);
            display.window = nullptr;
        }
    }
    if (gNanoVg) {
        nvgDeleteGL2(gNanoVg);
        gNanoVg = nullptr;
    }
}

PLUGIN_API int XPluginEnable(void) { return 1; }

PLUGIN_API void XPluginDisable(void) {}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int, void*) {}
