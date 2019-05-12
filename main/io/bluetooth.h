/*
 * author : Shuichi TAKANO
 * since  : Mon Dec 10 2018 1:52:46
 */
#ifndef _1B43F5B4_D133_F0D1_173F_43E96511E2E9
#define _1B43F5B4_D133_F0D1_173F_43E96511E2E9

namespace io
{

bool initializeBluetooth();
void setBluetoothDeviceName(const char* name);

void dumpBondedClassicBTDevices();
void removeAllBondedClassicBTDevices();

} // namespace io

#endif /* _1B43F5B4_D133_F0D1_173F_43E96511E2E9 */
