TARGET=astar
JS_OUT=$(TARGET).js

all:
	emcc $(TARGET).cpp -o $(JS_OUT) \
		-s EXPORTED_FUNCTIONS='["_aStar", "_aStarPathLength", "_transfer_params", "_malloc", "_free"]' \
		-s MODULARIZE=1 \
		-s EXPORT_NAME="createAStarModule" \
		-s ALLOW_MEMORY_GROWTH=1 \
		-s NO_DISABLE_EXCEPTION_CATCHING