< cdCommands

cd topbin
ld 0,0,"a16VmeTest.munch"
cd startup

## Register all support components
dbLoadDatabase("../../dbd/a16VmeTest.dbd",0,0)
a16VmeTest_registerRecordDeviceDriver(pdbbase)

## Configure A16VME board
## devA16VmeConfig(card,a16base,nreg,iVector,iLevel,iReg)
devA16VmeConfig(0,0x3600,8,0x00,5,7)

## a16Vme test database
dbLoadRecords("../../db/a16VmeTest.vdb","dev=a16Vme")

iocInit()
