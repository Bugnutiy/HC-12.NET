#pragma once

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG == 1
#define dd(X) Serial.print(X)
#define ddd(X) Serial.println(X)
#endif
