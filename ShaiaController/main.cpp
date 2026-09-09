#include <csignal>
#include <iostream>
#include <memory>

#include "Shaia.h"
#include "Def.h"
#include "ErrorDef.h"
#include "Logger.h"

std::unique_ptr<Shaia> pShaia;

void sigHandle(int sig) {
    (void)sig;
    std::cout << "Stopping...\n";
    if (pShaia) pShaia->stop();
}

int main() {
    signal(SIGINT, sigHandle);
    signal(SIGTERM, sigHandle);

    Logger::init(Logger::trace);
    LOG_INFO("Shaia Controller (Build 0.1.0)");
#ifdef SIMULATE
    LOG_INFO("*******SIMULATION MODE*******");
#endif

    pShaia = std::make_unique<Shaia>(PORT, HM_CONFIG_FILE, DANCE_MODES_FILE);

    Error_t e = pShaia->init(MASTER_HOST, MASTER_PORT);
    if (e != kNoError) {
        LOG_ERROR("Init failed. Error Code {}", (int)e);
        return (int)e;
    }

    pShaia->start();    // Blocks until a /shimon quit or a signal

    // Tear down here rather than at static destruction time - the serial and
    // Dynamixel singletons are function local statics and would already be
    // gone by then.
    pShaia.reset();

    return 0;
}
