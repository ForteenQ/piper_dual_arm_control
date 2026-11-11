import inspect
from piper_sdk import C_PiperInterface_V2
piper = C_PiperInterface_V2("can0")
print(inspect.getdoc(piper.MasterSlaveConfig))
