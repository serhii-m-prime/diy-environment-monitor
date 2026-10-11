#include <InputWaitPolicy.h>
#include <MonitorModel.h>
#include <cassert>
#include <cstdio>

int main() {
    monitor::InputWaitPolicy policy;
    assert(policy.observe(0, 3) == 10);
    assert(policy.observe(69, 3) == 10);
    assert(policy.observe(70, 3) == 1000);
    assert(policy.observe(5000, 2) == 10); // interrupt: left button pressed
    assert(policy.observe(5060, 2) == 10);
    assert(policy.observe(5070, 2) == 1000); // hold does not keep the CPU awake
    assert(policy.observe(9000, 6) == 10); // PIR rises during a held button
    assert(policy.observe(9070, 6) == 1000);
    assert(policy.observe(9080, 7) == 10); // button release wakes and debounces
    assert(!monitor::InputWaitPolicy::wakeOnHigh(true));
    assert(monitor::InputWaitPolicy::wakeOnHigh(false));
    monitor::InputWaitPolicy rollover;
    assert(rollover.observe(0xfffffff0U, 3) == 10);
    assert(rollover.observe(20, 3) == 10);
    assert(rollover.observe(54, 3) == 1000);
    assert(rollover.observe(55, 2) == 10);
    monitor::SampleSchedule schedule(0xfffffff0U);
    assert(schedule.untilTransition(0xfffffff0U) == 15000);
    assert(schedule.untilTransition(0xfffffff0U + 15000U) == 0);
    schedule.update(0xfffffff0U + 15500U); // late fan-off starts a complete settling pause
    assert(!schedule.fanNeeded() && schedule.untilTransition(0xfffffff0U + 15500U) == 2000);
    assert(schedule.untilTransition(0xfffffff0U + 17499U) == 1);
    assert(schedule.due(0xfffffff0U + 17500U));
    schedule.sampled(0xfffffff0U + 17500U);
    assert(schedule.untilTransition(0xfffffff0U + 17500U) == 43000);
    assert(schedule.untilTransition(0xfffffff0U + 60500U) == 0);
    std::puts("PASS: interrupt/debounce waits, held input wake levels, fan/settling deadlines and millis rollover");
}
