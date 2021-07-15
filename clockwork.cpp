#include "clockwork.h"

Clockwork::Clockwork(unsigned int period_ms) {
	_period_ms  = period_ms;
	_last_start = 0;
	_last_stop  = 0;
	_tet        = 0;
	_cb         = NULL;
}

Clockwork::Clockwork(unsigned int period_ms, CallbackType cb) {
	_period_ms  = period_ms;
	_last_start = 0;
	_last_stop  = 0;
	_tet        = 0;
	_cb         = cb;
}

void Clockwork::start() {
	_last_start = micros();
}

bool Clockwork::stop() {
	_last_stop = micros();
	
	if(_last_stop < _last_start){ // OVERFLOW!
		micros_t max = -1;
		_tet = _last_stop + (max - _last_start);
	} else {
		_tet = _last_stop - _last_start;
	}

	long int time_to_wait = (1000 * _period_ms) - _tet;

	if(time_to_wait < 0) {
		if(_cb != NULL) {
			_cb(_tet);
		}
		return false;
		Serial.println("tet alarm");
	}

	if(time_to_wait > 1000) {
		delay(time_to_wait / 1000);
	}

	delayMicroseconds(time_to_wait % 1000);

	return true;

}




