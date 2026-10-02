# Product deployment entry point

The XGC2 product owns `xgc-rt-render`, its robot recipes and its deployment
bundle contract in `robot-container/fs150/native-deployment` of the XGC2
product repository. Build it through the existing product/onboard packaging
entry point. There is one renderer implementation and one CLI name.

`xgc-rt-host` only accepts a generic manifest and artifact pins. It does not
select PX4, DMPC or HIL compositions or interpret robot parameters. Generic
manifest and clock behavior are documented in [the time model](time-model.md).
