TEMPLATE_DIR=models/tinyhawk
CFG=$TEMPLATE_DIR/aircraft.json
python3 compute_inertia.py --template-dir $TEMPLATE_DIR  --aircraft-config $CFG
