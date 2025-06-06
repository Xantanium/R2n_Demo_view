/**
 * @file Scheduler.h
 * @brief Scheduler class definition.
 * @author Xantanium
 */

#pragma once
#include <cstddef>
#include "../Task/Task.h"

/**
 * @class Scheduler
 * @brief Manages Periodic and perpetual task execution both in the main loop and separate threads.
 */
class Scheduler {
public:
    /**
     * @brief default constructor
     */
    Scheduler() = default;

    /**
     * @brief Thread entry point function
     *
     * This function each threaded task will execute.
     * It receives a pointer to a task object and runs it in a loop.
     * A separate wrapper function is required since the addThread function does not accept lambdas.
     * @param arg void a pointer to the task.
     */
    [[noreturn]] static void threadRunner(void *arg);

    /**
     * @brief adds a task to the scheduler execution pipeline.
     *
     * @note A task will run perpetually if the interval is set to 0.
     *
     * @param callback function called by the scheduler
     * @param interval millisecond interval between consequent executions.
     * @param useThread should the function be offloaded to separate thread?
     */
    void addTask(
            const Task::TaskCallback &callback, unsigned long interval, bool useThread = false);

    /**
     * @brief updates all non-threaded tasks.
     *
     * @note This function should be called in the main loop.
     */
    void update();

    /**
     * @brief enable a task by index.
     * @param taskIndex index in the vector of tasks.
     */
    inline void enableTask(const std::size_t taskIndex) {
        if (taskIndex < tasks.size())
            tasks[taskIndex].isEnabled = true;
    }

    /**
     * @brief disable a task by index.
     * @param taskIndex index in the vector of tasks.
     */
    inline void disableTask(const std::size_t taskIndex) {
        if (taskIndex < tasks.size())
            tasks[taskIndex].isEnabled = false;
    }

    /**
     * @brief get the number of tasks in the scheduler.
     * @return number of tasks in the scheduler.
     */
    [[nodiscard]] std::size_t getTaskCount() const { return tasks.size(); }
private:
    std::vector<Task> tasks; ///< vector of tasks
};
