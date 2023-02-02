#include "Arduino.h"
// #define BUFFER_SIZE 5

// For printing
#include <iostream>
using namespace std;

template <typename datatype, int BUFFER_SIZE>
class circular_buffer{
	public:
		int _addIndex = 0;
		int _oldestIndex = 0;
		datatype _buffer[BUFFER_SIZE];

		void add(datatype value) {
			if (_addIndex >= BUFFER_SIZE) {
				_oldestIndex = _addIndex % BUFFER_SIZE + 1;
			}
			_buffer[_addIndex % BUFFER_SIZE] = value;
			_addIndex++;
		}

		void print_array() {
			Serial.print("[");
			for (int i = 0; i < BUFFER_SIZE; i++) {
				Serial.print(_buffer[i]);
				Serial.print(" ");
			}
			Serial.println("]");
		}

		datatype average_value() {
			datatype _sum = 0;
			for (int i = 0; i < BUFFER_SIZE; i++) {
				_sum += _buffer[i];
			}
			return _sum / BUFFER_SIZE;
		}

		//C++ Standard Printing
		// void print_array() {
		// 	cout << "[ ";
		// 	for (int i = 0; i < BUFFER_SIZE; i++) {
		// 		cout << _buffer[i] << " ";
		// 	}
		// 	cout << "]" << "\n";
		// }

		// void print_sequential() {
		// 	cout << "[ ";
		// 	for (int i = _oldestIndex; i < BUFFER_SIZE; i++) {
		// 		cout << _buffer[i] << " ";
		// 	}
		// 	for (int i = 0; i < _oldestIndex; i++) {
		// 		cout << _buffer[i] << " ";
		// 	}
		// 	cout << "]" << "\n";
		// }
};