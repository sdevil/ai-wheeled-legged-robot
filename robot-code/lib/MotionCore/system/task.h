#ifndef TASK_H
#define TASK_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

class task {
	public:
		// rtos 任务构造函数
		task(uint32_t loop_time,
			void (*func)(uint32_t),
			uint32_t stack_depth,
			UBaseType_t priority,
			BaseType_t core_id);

		// esp_timer 任务构造函数
		task(uint32_t loop_time,
			void (*func)(uint32_t));

	public:
		void start();

	private:
		enum class type_t {
			rtos,
			esp_timer
		};

		type_t type;
		uint32_t loop_time;
		void (*task_func)(uint32_t);
		uint32_t stack_depth;
		UBaseType_t priority;
		BaseType_t core_id;
		esp_timer_handle_t xTimer;

	private:
		static void rtos_entry(void *arg);
		static void timer_callback(void *arg);
};

#endif
