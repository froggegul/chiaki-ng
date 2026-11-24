// SPDX-License-Identifier: LicenseRef-AGPL-3.0-only-OpenSSL
// Mouse-aim / keyboard-to-controller layer for macOS (chiaki-ng KBM edition).
#ifndef CHIAKI_KBMAIM_H
#define CHIAKI_KBMAIM_H

#include <chiaki/controller.h>
#include <functional>

class KbmAimPrivate;

/**
 * Captures keyboard + mouse while the chiaki-ng window is active and turns them
 * into a DualSense state: WASD -> left stick, mouse movement -> right stick
 * (with sensitivity / response curve / anti-deadzone / smoothing), buttons by
 * physical key position (works with Korean / any input source).
 * Toggle with ` (backtick, left of 1) or F8. Caps Lock switches foot/vehicle layout.
 */
class KbmAim
{
public:
	KbmAim();
	~KbmAim();

	void SetEnabled(bool enabled);
	bool IsEnabled() const;
	/** Called when the state changed and should be sent. */
	void SetChangedCallback(std::function<void()> cb);
	/** Advance aim smoothing; returns true if the state changed. */
	bool Tick();
	const ChiakiControllerState &State() const;

private:
	KbmAimPrivate *d;
};

#endif
