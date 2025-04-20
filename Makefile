TARGET=astar
JS_OUT=$(TARGET).js

all:
	emcc $(TARGET).cpp -o $(JS_OUT) \
		-s EXPORTED_FUNCTIONS='["_astar"]' \
		-s EXPORTED_RUNTIME_METHODS='["cwrap"]' \
		-s MODULARIZE=1 \
		-s EXPORT_NAME="createAStarModule"