# F28HS Coursework 1

H00391693: HSHEX MEDIAN HIST

## Stage 1: Compile C Codes

To compile the program, simply run the make command in your terminal:

```
make
```

When it has been compiled successfully, the below command will be shown in your terminal. 

```
cc -std=c99 -O2 -Wall -pedantic -g -o process process.c
```

## Stage 2: Process Original Image

### Processing a Single Image

To process a single HSHEX image, use the following command format:

```
./process imput.hshex input_out.hshex
```

Replace input.hsdec with the path to your input image file in HSHEX format, and output.hshex with the desired output file path.

### Processing Multiple Images

To process multiple images in succession, provide the input and output file paths alternately. For example:

```
./process input1.hshex input1_out.hshex input2.hshex input2_out.hshex ...
```

Ensure that you provide an even number of command-line arguments, with each pair representing an input-output file pair.

### Example

Here's an example of how to use the program to process a single image:

```
./process bars.hshex bars_out.hshex
```

And here's an example of processing multiple images:

```
./process bars.hsdex bars_out.hsdex coffee.hshex coffee_out.hshex music.hshex music_out.hshex
```

## Stage 3: Check Processed Image

### Convert personalised format into .ppm;

There is a python code called hsconvert, which can convert personalised image format into .ppm format that can be easy to reveal and check in the Linux machine. To convert your image, use the following command format:

```
./hsconvert input_out.hshex input_out.ppm
```

### Example

Here's an example of how to use the program to check processed image by using hsconvert, .ppm file can be seen in Linux machine:

```
./hsconvert bars_out.hshex bars_out.ppm
```
