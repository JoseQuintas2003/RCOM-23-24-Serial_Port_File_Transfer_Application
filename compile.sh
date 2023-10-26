gcc main.c src/link_layer.c src/application_layer.c -o main -lm
cp main Receiver
cp main Transmitter
