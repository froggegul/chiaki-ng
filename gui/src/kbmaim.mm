// SPDX-License-Identifier: LicenseRef-AGPL-3.0-only-OpenSSL
// Mouse-aim / keyboard-to-controller layer for macOS (chiaki-ng KBM edition).

#include "kbmaim.h"

#import <AppKit/AppKit.h>
#import <ApplicationServices/ApplicationServices.h>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>
#include <QStringList>
#include <QMap>
#include <QSet>

#include <chrono>
#include <cmath>
#include <algorithm>

namespace {

// macOS virtual key codes (physical positions, independent of the input source).
const QMap<QString, int> &KeyCodes()
{
	static const QMap<QString, int> m = {
		{"A", 0}, {"S", 1}, {"D", 2}, {"F", 3}, {"H", 4}, {"G", 5}, {"Z", 6}, {"X", 7}, {"C", 8}, {"V", 9},
		{"B", 11}, {"Q", 12}, {"W", 13}, {"E", 14}, {"R", 15}, {"Y", 16}, {"T", 17}, {"1", 18}, {"2", 19},
		{"3", 20}, {"4", 21}, {"6", 22}, {"5", 23}, {"=", 24}, {"9", 25}, {"7", 26}, {"-", 27}, {"8", 28},
		{"0", 29}, {"]", 30}, {"O", 31}, {"U", 32}, {"[", 33}, {"I", 34}, {"P", 35}, {"Return", 36},
		{"L", 37}, {"J", 38}, {"'", 39}, {"K", 40}, {";", 41}, {"\\", 42}, {",", 43}, {"/", 44}, {"N", 45},
		{"M", 46}, {".", 47}, {"Tab", 48}, {"Space", 49}, {"`", 50}, {"Backspace", 51}, {"Esc", 53},
		{"RightCmd", 54}, {"Cmd", 55}, {"Shift", 56}, {"CapsLock", 57}, {"Option", 58}, {"Control", 59},
		{"RightShift", 60}, {"RightOption", 61}, {"RightControl", 62},
		{"F5", 96}, {"F6", 97}, {"F7", 98}, {"F3", 99}, {"F8", 100}, {"F9", 101}, {"F11", 103}, {"F10", 109},
		{"F12", 111}, {"F4", 118}, {"F2", 120}, {"F1", 122},
		{"Left", 123}, {"Right", 124}, {"Down", 125}, {"Up", 126},
	};
	return m;
}

// Pseudo codes for mouse inputs.
enum { MOUSE_LEFT = 1000, MOUSE_RIGHT, MOUSE_MIDDLE, MOUSE_4, MOUSE_5, WHEEL_UP, WHEEL_DOWN };

int CodeForName(const QString &name)
{
	static const QMap<QString, int> mouse = {
		{"MouseLeft", MOUSE_LEFT}, {"MouseRight", MOUSE_RIGHT}, {"MouseMiddle", MOUSE_MIDDLE},
		{"Mouse4", MOUSE_4}, {"Mouse5", MOUSE_5}, {"WheelUp", WHEEL_UP}, {"WheelDown", WHEEL_DOWN},
	};
	if(mouse.contains(name))
		return mouse[name];
	auto &k = KeyCodes();
	auto it = k.find(name.length() == 1 ? name.toUpper() : name);
	return it == k.end() ? -1 : it.value();
}

const int KEY_TOGGLE_1 = 50;   // `
const int KEY_TOGGLE_2 = 100;  // F8
const int KEY_CAPS = 57;
const int KEY_SENS_DOWN = 27, KEY_SENS_UP = 24;      // - =
const int KEY_ADZ_DOWN = 33, KEY_ADZ_UP = 30;        // [ ]

QJsonObject DefaultFoot()
{
	return QJsonObject{
		{"W", "LUp"}, {"S", "LDown"}, {"A", "LLeft"}, {"D", "LRight"},
		{"MouseLeft", "R2"}, {"MouseRight", "L2"},
		{"Shift", "Cross"}, {"Space", "Square"}, {"F", "Triangle"}, {"R", "Circle"},
		{"Q", "R1"}, {"Tab", "L1"}, {"Mouse4", "L1"}, {"Mouse5", "R1"},
		{"Control", "L3"}, {"C", "R3"}, {"MouseMiddle", "R3"},
		{"E", "DpadRight"}, {"Up", "DpadUp"}, {"Down", "DpadDown"}, {"Left", "DpadLeft"}, {"Right", "DpadRight"},
		{"1", "DpadUp"}, {"2", "DpadDown"}, {"3", "DpadLeft"}, {"4", "DpadRight"},
		{"M", "Touchpad"}, {"Esc", "Options"}, {"F12", "Share"}, {"P", "PS"},
		{"Option", "Walk"},
		{"I", "RUp"}, {"K", "RDown"}, {"J", "RLeft"}, {"L", "RRight"},
	};
}

QJsonObject DefaultVehicle()
{
	return QJsonObject{ {"W", "R2"}, {"S", "L2"}, {"Space", "R1"}, {"H", "L3"} };
}

uint32_t ButtonFor(const QString &o)
{
	static const QMap<QString, uint32_t> m = {
		{"Cross", CHIAKI_CONTROLLER_BUTTON_CROSS}, {"Circle", CHIAKI_CONTROLLER_BUTTON_MOON},
		{"Square", CHIAKI_CONTROLLER_BUTTON_BOX}, {"Triangle", CHIAKI_CONTROLLER_BUTTON_PYRAMID},
		{"DpadLeft", CHIAKI_CONTROLLER_BUTTON_DPAD_LEFT}, {"DpadRight", CHIAKI_CONTROLLER_BUTTON_DPAD_RIGHT},
		{"DpadUp", CHIAKI_CONTROLLER_BUTTON_DPAD_UP}, {"DpadDown", CHIAKI_CONTROLLER_BUTTON_DPAD_DOWN},
		{"L1", CHIAKI_CONTROLLER_BUTTON_L1}, {"R1", CHIAKI_CONTROLLER_BUTTON_R1},
		{"L3", CHIAKI_CONTROLLER_BUTTON_L3}, {"R3", CHIAKI_CONTROLLER_BUTTON_R3},
		{"Options", CHIAKI_CONTROLLER_BUTTON_OPTIONS}, {"Share", CHIAKI_CONTROLLER_BUTTON_SHARE},
		{"Touchpad", CHIAKI_CONTROLLER_BUTTON_TOUCHPAD}, {"PS", CHIAKI_CONTROLLER_BUTTON_PS},
	};
	return m.value(o, 0);
}

int16_t ToAxis(double v)
{
	v = std::clamp(v, -1.0, 1.0);
	return (int16_t)std::lround(v * 32767.0);
}

} // namespace

class KbmAimPrivate
{
public:
	// settings
	double sensitivity = 1.0, y_ratio = 1.0, curve = 0.8, anti_deadzone = 0.15, smoothing_ms = 8.0, walk_scale = 0.45;
	bool invert_y = false, sounds = true;
	QMap<int, QStringList> foot, vehicle;
	QString config_path;

	// runtime
	bool enabled = false;
	bool vehicle_mode = false;
	bool captured = false;
	id monitor = nil;
	id obs_active = nil, obs_resign = nil;
	QSet<int> held;
	QMap<int, std::chrono::steady_clock::time_point> pulses;
	double acc_x = 0, acc_y = 0;
	double vx = 0, vy = 0, idle = 0, stick_x = 0, stick_y = 0;
	std::chrono::steady_clock::time_point last_tick = std::chrono::steady_clock::now();
	ChiakiControllerState state;
	std::function<void()> changed;

	KbmAimPrivate()
	{
		chiaki_controller_state_set_idle(&state);
		config_path = QDir::homePath() + "/Library/Application Support/chiaki-kbm/settings.json";
		LoadConfig();
	}

	static QMap<int, QStringList> ParseMap(const QJsonObject &o)
	{
		QMap<int, QStringList> m;
		for(auto it = o.begin(); it != o.end(); ++it)
		{
			int code = CodeForName(it.key());
			if(code < 0)
				continue;
			QStringList outs = it.value().toString().split('+', Qt::SkipEmptyParts);
			for(auto &s : outs) s = s.trimmed();
			m[code] = outs;
		}
		return m;
	}

	QJsonObject ToJson(const QJsonObject &footJ, const QJsonObject &vehJ)
	{
		return QJsonObject{
			{"_help", "Keys: letters, 0-9, Space, Shift, Control, Option, Tab, Esc, Return, Up/Down/Left/Right, F1-F12, MouseLeft, MouseRight, MouseMiddle, Mouse4, Mouse5, WheelUp, WheelDown. "
			          "Outputs: Cross Circle Square Triangle L1 R1 L2 R2 L3 R3 DpadUp DpadDown DpadLeft DpadRight Options Share Touchpad PS LUp LDown LLeft LRight RUp RDown RLeft RRight Walk (combine with +). "
			          "'vehicle' only lists keys that differ from 'foot'."},
			{"sensitivity", sensitivity}, {"vertical_ratio", y_ratio}, {"response_curve", curve},
			{"anti_deadzone", anti_deadzone}, {"smoothing_ms", smoothing_ms}, {"invert_y", invert_y},
			{"walk_scale", walk_scale}, {"sounds", sounds},
			{"foot", footJ}, {"vehicle", vehJ},
		};
	}

	QJsonObject foot_json, vehicle_json;

	void LoadConfig()
	{
		QFile f(config_path);
		QJsonObject o;
		if(f.open(QIODevice::ReadOnly))
			o = QJsonDocument::fromJson(f.readAll()).object();
		sensitivity = o.value("sensitivity").toDouble(sensitivity);
		y_ratio = o.value("vertical_ratio").toDouble(y_ratio);
		curve = o.value("response_curve").toDouble(curve);
		anti_deadzone = o.value("anti_deadzone").toDouble(anti_deadzone);
		smoothing_ms = o.value("smoothing_ms").toDouble(smoothing_ms);
		invert_y = o.value("invert_y").toBool(invert_y);
		walk_scale = o.value("walk_scale").toDouble(walk_scale);
		sounds = o.value("sounds").toBool(sounds);
		foot_json = o.value("foot").isObject() ? o.value("foot").toObject() : DefaultFoot();
		vehicle_json = o.value("vehicle").isObject() ? o.value("vehicle").toObject() : DefaultVehicle();
		foot = ParseMap(foot_json);
		vehicle = ParseMap(vehicle_json);
		if(!f.exists())
			SaveConfig();
	}

	void SaveConfig()
	{
		QDir().mkpath(QFileInfo(config_path).absolutePath());
		QFile f(config_path);
		if(f.open(QIODevice::WriteOnly | QIODevice::Truncate))
			f.write(QJsonDocument(ToJson(foot_json, vehicle_json)).toJson(QJsonDocument::Indented));
	}

	void Sound(NSString *name)
	{
		if(!sounds)
			return;
		NSSound *s = [NSSound soundNamed:name];
		[s stop];
		[s play];
	}

	bool IsBound(int code) const
	{
		auto fv = foot.find(code);
		if(fv != foot.end() && !fv.value().isEmpty()) return true;
		auto vv = vehicle.find(code);
		return vv != vehicle.end() && !vv.value().isEmpty();
	}

	void Capture(bool on)
	{
		if(on == captured)
			return;
		captured = on;
		CGAssociateMouseAndMouseCursorPosition(on ? false : true);
		if(on) [NSCursor hide]; else [NSCursor unhide];
		if(!on)
		{
			held.clear();
			pulses.clear();
			acc_x = acc_y = vx = vy = stick_x = stick_y = 0;
		}
	}

	bool ShouldCapture() const { return enabled && [NSApp isActive]; }

	void Toggle()
	{
		enabled = !enabled;
		Capture(ShouldCapture());
		Sound(enabled ? @"Tink" : @"Pop");
		Notify();
	}

	void Notify() { if(changed) changed(); }

	// Returns nil to swallow the event.
	NSEvent *Handle(NSEvent *e)
	{
		NSEventType t = e.type;
		if(t == NSEventTypeKeyDown || t == NSEventTypeKeyUp)
		{
			int code = e.keyCode;
			bool down = t == NSEventTypeKeyDown;
			if(code == KEY_TOGGLE_1 || code == KEY_TOGGLE_2)
			{
				if(down && !e.isARepeat && !(e.modifierFlags & NSEventModifierFlagCommand))
					Toggle();
				return (e.modifierFlags & NSEventModifierFlagCommand) ? e : nil;
			}
			if(!captured)
				return e;
			if(e.modifierFlags & NSEventModifierFlagCommand)
				return e; // keep Cmd+Q, Cmd+H, ... working
			if(down && !e.isARepeat)
			{
				bool changedCfg = true;
				if(code == KEY_SENS_UP && !IsBound(code)) sensitivity = std::min(10.0, sensitivity * 1.1);
				else if(code == KEY_SENS_DOWN && !IsBound(code)) sensitivity = std::max(0.05, sensitivity / 1.1);
				else if(code == KEY_ADZ_UP && !IsBound(code)) anti_deadzone = std::min(0.6, anti_deadzone + 0.02);
				else if(code == KEY_ADZ_DOWN && !IsBound(code)) anti_deadzone = std::max(0.0, anti_deadzone - 0.02);
				else changedCfg = false;
				if(changedCfg) { Sound(@"Morse"); SaveConfig(); return nil; }
			}
			if(down) held.insert(code); else held.remove(code);
			Notify();
			return nil;
		}
		if(t == NSEventTypeFlagsChanged)
		{
			if(!captured)
				return e;
			int code = e.keyCode;
			NSEventModifierFlags f = e.modifierFlags;
			if(code == KEY_CAPS)
			{
				vehicle_mode = !vehicle_mode;
				Sound(vehicle_mode ? @"Submarine" : @"Bottle");
				Notify();
				return nil;
			}
			bool down;
			switch(code)
			{
				case 56: case 60: down = f & NSEventModifierFlagShift; break;
				case 59: case 62: down = f & NSEventModifierFlagControl; break;
				case 58: case 61: down = f & NSEventModifierFlagOption; break;
				default: return e; // Cmd etc.
			}
			if(down) held.insert(code); else held.remove(code);
			Notify();
			return nil;
		}
		if(!captured)
			return e;
		switch(t)
		{
			case NSEventTypeMouseMoved:
			case NSEventTypeLeftMouseDragged:
			case NSEventTypeRightMouseDragged:
			case NSEventTypeOtherMouseDragged:
				acc_x += e.deltaX;
				acc_y += e.deltaY;
				return nil;
			case NSEventTypeLeftMouseDown: held.insert(MOUSE_LEFT); Notify(); return nil;
			case NSEventTypeLeftMouseUp: held.remove(MOUSE_LEFT); Notify(); return nil;
			case NSEventTypeRightMouseDown: held.insert(MOUSE_RIGHT); Notify(); return nil;
			case NSEventTypeRightMouseUp: held.remove(MOUSE_RIGHT); Notify(); return nil;
			case NSEventTypeOtherMouseDown:
			case NSEventTypeOtherMouseUp:
			{
				int b = (int)e.buttonNumber;
				int code = b == 2 ? MOUSE_MIDDLE : b == 3 ? MOUSE_4 : b == 4 ? MOUSE_5 : -1;
				if(code < 0) return nil;
				if(t == NSEventTypeOtherMouseDown) held.insert(code); else held.remove(code);
				Notify();
				return nil;
			}
			case NSEventTypeScrollWheel:
			{
				double dy = e.scrollingDeltaY;
				if(std::abs(dy) < 0.5 || e.momentumPhase != NSEventPhaseNone)
					return nil;
				int code = dy > 0 ? WHEEL_UP : WHEEL_DOWN;
				pulses[code] = std::chrono::steady_clock::now() + std::chrono::milliseconds(70);
				Notify();
				return nil;
			}
			default:
				return e;
		}
	}

	void UpdateAim(double dx, double dy, double dt)
	{
		if(dt <= 0) dt = 0.001;
		if(dt > 0.1) dt = 0.1;
		if(dx == 0 && dy == 0)
		{
			idle += dt;
			if(idle >= std::max(0.012, 1.5 * smoothing_ms / 1000.0)) { vx = vy = stick_x = stick_y = 0; return; }
		}
		else
			idle = 0;
		const double base_full_speed = 1500.0; // macOS deltas are in points (already scaled), lower than raw counts
		double tau = std::max(0.0, smoothing_ms) / 1000.0;
		double a = tau <= 0 ? 1.0 : 1.0 - std::exp(-dt / tau);
		vx += a * (dx / dt - vx);
		vy += a * (dy / dt - vy);
		double sx = vx * sensitivity / base_full_speed;
		double sy = vy * sensitivity * y_ratio / base_full_speed;
		if(invert_y) sy = -sy;
		double m = std::sqrt(sx * sx + sy * sy);
		if(m < 0.004) { stick_x = stick_y = 0; return; }
		double shaped = std::pow(std::min(m, 1.0), std::max(0.05, curve));
		double adz = std::clamp(anti_deadzone, 0.0, 0.9);
		double out = std::min(1.0, adz + (1.0 - adz) * shaped);
		stick_x = sx / m * out;
		stick_y = sy / m * out;
	}

	bool Tick()
	{
		auto now = std::chrono::steady_clock::now();
		double dt = std::chrono::duration<double>(now - last_tick).count();
		last_tick = now;

		bool want = ShouldCapture();
		if(want != captured)
			Capture(want);

		ChiakiControllerState s;
		chiaki_controller_state_set_idle(&s);
		if(captured)
		{
			double dx = acc_x, dy = acc_y;
			acc_x = acc_y = 0;
			UpdateAim(dx, dy, dt);

			for(auto it = pulses.begin(); it != pulses.end();)
				it = it.value() < now ? pulses.erase(it) : std::next(it);

			QSet<QString> outs;
			auto collect = [&](int code) {
				if(vehicle_mode && vehicle.contains(code)) { for(auto &o : vehicle[code]) outs.insert(o); }
				else if(foot.contains(code)) { for(auto &o : foot[code]) outs.insert(o); }
			};
			for(int c : held) collect(c);
			for(auto it = pulses.begin(); it != pulses.end(); ++it) collect(it.key());

			double lx = (outs.contains("LRight") ? 1 : 0) - (outs.contains("LLeft") ? 1 : 0);
			double ly = (outs.contains("LDown") ? 1 : 0) - (outs.contains("LUp") ? 1 : 0);
			double lm = std::sqrt(lx * lx + ly * ly);
			if(lm > 1) { lx /= lm; ly /= lm; }
			if(outs.contains("Walk")) { lx *= walk_scale; ly *= walk_scale; }
			double rx = stick_x + (outs.contains("RRight") ? 1 : 0) - (outs.contains("RLeft") ? 1 : 0);
			double ry = stick_y + (outs.contains("RDown") ? 1 : 0) - (outs.contains("RUp") ? 1 : 0);
			double rm = std::sqrt(rx * rx + ry * ry);
			if(rm > 1) { rx /= rm; ry /= rm; }

			s.left_x = ToAxis(lx); s.left_y = ToAxis(ly);
			s.right_x = ToAxis(rx); s.right_y = ToAxis(ry);
			s.l2_state = outs.contains("L2") ? 0xff : 0;
			s.r2_state = outs.contains("R2") ? 0xff : 0;
			for(auto &o : outs) s.buttons |= ButtonFor(o);
			// opposite d-pad directions cancel
			if((s.buttons & CHIAKI_CONTROLLER_BUTTON_DPAD_UP) && (s.buttons & CHIAKI_CONTROLLER_BUTTON_DPAD_DOWN))
				s.buttons &= ~(CHIAKI_CONTROLLER_BUTTON_DPAD_UP | CHIAKI_CONTROLLER_BUTTON_DPAD_DOWN);
			if((s.buttons & CHIAKI_CONTROLLER_BUTTON_DPAD_LEFT) && (s.buttons & CHIAKI_CONTROLLER_BUTTON_DPAD_RIGHT))
				s.buttons &= ~(CHIAKI_CONTROLLER_BUTTON_DPAD_LEFT | CHIAKI_CONTROLLER_BUTTON_DPAD_RIGHT);
		}
		bool diff = s.buttons != state.buttons || s.l2_state != state.l2_state || s.r2_state != state.r2_state
			|| s.left_x != state.left_x || s.left_y != state.left_y || s.right_x != state.right_x || s.right_y != state.right_y;
		if(diff)
		{
			state.buttons = s.buttons; state.l2_state = s.l2_state; state.r2_state = s.r2_state;
			state.left_x = s.left_x; state.left_y = s.left_y; state.right_x = s.right_x; state.right_y = s.right_y;
		}
		return diff;
	}
};

KbmAim::KbmAim() : d(new KbmAimPrivate)
{
	KbmAimPrivate *p = d;
	NSEventMask mask = NSEventMaskKeyDown | NSEventMaskKeyUp | NSEventMaskFlagsChanged |
		NSEventMaskMouseMoved | NSEventMaskLeftMouseDragged | NSEventMaskRightMouseDragged | NSEventMaskOtherMouseDragged |
		NSEventMaskLeftMouseDown | NSEventMaskLeftMouseUp | NSEventMaskRightMouseDown | NSEventMaskRightMouseUp |
		NSEventMaskOtherMouseDown | NSEventMaskOtherMouseUp | NSEventMaskScrollWheel;
	d->monitor = [NSEvent addLocalMonitorForEventsMatchingMask:mask handler:^NSEvent *(NSEvent *e) {
		return p->Handle(e);
	}];
	NSNotificationCenter *nc = [NSNotificationCenter defaultCenter];
	d->obs_resign = [nc addObserverForName:NSApplicationDidResignActiveNotification object:nil queue:nil usingBlock:^(NSNotification *) {
		p->Capture(false);
		p->Tick();
		p->Notify();
	}];
	d->obs_active = [nc addObserverForName:NSApplicationDidBecomeActiveNotification object:nil queue:nil usingBlock:^(NSNotification *) {
		p->Capture(p->ShouldCapture());
	}];
}

KbmAim::~KbmAim()
{
	if(d->monitor) [NSEvent removeMonitor:d->monitor];
	NSNotificationCenter *nc = [NSNotificationCenter defaultCenter];
	if(d->obs_resign) [nc removeObserver:d->obs_resign];
	if(d->obs_active) [nc removeObserver:d->obs_active];
	d->Capture(false);
	delete d;
}

void KbmAim::SetEnabled(bool enabled)
{
	if(d->enabled == enabled)
		return;
	d->Toggle();
}

bool KbmAim::IsEnabled() const { return d->enabled; }
void KbmAim::SetChangedCallback(std::function<void()> cb) { d->changed = std::move(cb); }
bool KbmAim::Tick() { return d->Tick(); }
const ChiakiControllerState &KbmAim::State() const { return d->state; }
