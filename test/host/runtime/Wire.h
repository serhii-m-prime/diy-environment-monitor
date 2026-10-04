#pragma once
struct MockWire { void begin(int,int) {} void setTimeOut(int) {} };
inline MockWire Wire;
