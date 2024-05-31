/* This coursework specification, and the example code provided during the
 * course, is Copyright 2024 Heriot-Watt University.
 * Distributing this coursework specification or your solution to it outside
 * the university is academic misconduct and a violation of copyright law. */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* The RGB values of a pixel. */
struct Pixel
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
};

/* An image loaded from a file. */
typedef struct Image
{
    int width;
    int height;
    struct Pixel *pixels;
    // add it for usingt linklist
    char *name;
} Image;

/* Define the linklist. */
typedef struct LinkList
{
    Image *image;
    struct LinkList *next;
} LinkList;

// Function to compare two uint16_t values for qsort
int compare(const void *x, const void *y)
{
    return (*(uint16_t *)x - *(uint16_t *)y);
}

// Function to calculate the average of an array of uint16_t values
uint16_t calculate_average(uint16_t *values, int count)
{
    uint32_t sum = 0;

    for (int i = 0; i < count; i++)
    {
        sum += values[i];
    }

    return (uint16_t)(sum / count);
}

/* Free a struct Image */
void free_image(struct Image *img)
{
    if (img != NULL)
    {
        free(img->pixels);
        free(img);
    }
}

/* Opens and reads an image file, returning a pointer to a new struct Image.
 * On error, prints an error message and returns NULL. */
struct Image *load_image(const char *filename)
{
    /* Open the file for reading */
    FILE *f = fopen(filename, "r");
    if (f == NULL)
    {
        fprintf(stderr, "File %s could not be opened.\n", filename);
        return NULL;
    }

    /* Allocate the Image object, and read the image from the file and check the malloc */
    struct Image *img = malloc(sizeof(struct Image));
    if (img == NULL)
    {
        fprintf(stderr, "Memory allocation failed.\n");
        fclose(f);
        return NULL;
    }

    int width, height;

    // Read image width and height from file
    if (fscanf(f, "HSHEX %d %d ", &width, &height) != 2)
    {
        fprintf(stderr, "Invalid image format.\n");
        fclose(f);
        return NULL;
    }

    // Allocate memory for image pixels and check the malloc
    img->pixels = malloc(width * height * sizeof(struct Pixel));
    if (img->pixels == NULL)
    {
        fprintf(stderr, "Invalid image format.\n");
        free(img);
        return NULL;
    }

    // Set image width and height
    img->width = width;
    img->height = height;

    // Read pixel data from file
    for (int i = 0; i < width * height; i++)
    {
        if (fscanf(f, "%hx %hx %hx", &img->pixels[i].red, &img->pixels[i].green, &img->pixels[i].blue) != 3)
        {
            fprintf(stderr, "Invalid image format.\n");
            fclose(f);
            return NULL;
        }
    }

    return img;
}

// bool save_image(const struct Image *img, const char *filename)
/* Write img to file filename. Return true on success, false on error. */
// HSHEX
bool save_image(const struct Image *img)
{
    // Open file for writing and check it
    FILE *f = fopen(img->name, "w");
    if (f == NULL)
    {
        return false;
    }

    int width = img->width;
    int height = img->height;

    // Write image width and height to file
    if (fprintf(f, "HSHEX %d %d ", width, height) < 0)
    {
        fprintf(stderr, "Error writing image data to file.\n");
        return false;
    }

    // Write pixel data to file
    for (int i = 0; i < img->width * img->height; i++)
    {
        if (fprintf(f, "%hx %hx %hx ", img->pixels[i].red, img->pixels[i].green, img->pixels[i].blue) < 0)
        {
            fprintf(stderr, "Error writing image data to file.\n");
            return false;
        }
    }

    // close file and check it
    if (fclose(f) != 0)
    {
        fprintf(stderr, "Error closing the file.\n");
        return false;
    }

    return true;
}

/* Allocate a new struct Image and copy an existing struct Image's contents
 * into it. On error, returns NULL. */
struct Image *copy_image(const struct Image *source)
{
    // Allocate memory for the new image structure and check the malloc
    struct Image *img = malloc(sizeof(struct Image));
    if (img == NULL)
    {
        fprintf(stderr, "Memory allocation failed.\n");
        return NULL;
    }

    // Copy width and height from the source image
    img->width = source->width;
    img->height = source->height;

    // Allocate memory for the pixels of the new image
    img->pixels = malloc(source->width * source->height * sizeof(struct Pixel));
    if (img->pixels == NULL)
    {
        fprintf(stderr, "Memory allocation failed.\n");
        free(img);
        return NULL;
    }

    // Copy pixel data from the source image to the new image
    for (int i = 0; i < source->width * source->height; i++)
    {
        img->pixels[i].red = source->pixels[i].red;
        img->pixels[i].green = source->pixels[i].green;
        img->pixels[i].blue = source->pixels[i].blue;
    }

    return img;
}

/* Perform your first task.
 * (TODO: Write a better comment here, and rename the function.
 * You may need to add or change arguments depending on the task.)
 * Returns a new struct Image containing the result, or NULL on error. */
struct Image *apply_MEDIAN(const struct Image *source)
{
    // Create a copy of the source image
    struct Image *result = copy_image(source);
    if (result == NULL)
    {
        fprintf(stderr, "Memory allocation fail.\n");
        return NULL;
    }

    // Copy width and height
    result->width = source->width;
    result->height = source->height;

    // Allocate memory for the pixel array
    result->pixels = malloc(source->width * source->height * sizeof(struct Pixel));
    if (result->pixels == NULL)
    {
        fprintf(stderr, "Memory allocation failed.\n");
        free(result);
        return NULL;
    }

    // Iterate over each pixel in the image
    for (int i = 0; i < source->height; i++)
    {
        for (int j = 0; j < source->width; j++)
        {
            int count = 0;
            struct Pixel neighbors[4];

            // Get neighboring pixels
            if (i > 0)
            {
                neighbors[count++] = source->pixels[(i - 1) * source->width + j];
            }
            if (i < source->height - 1)
            {
                neighbors[count++] = source->pixels[(i + 1) * source->width + j];
            }
            if (j > 0)
            {
                neighbors[count++] = source->pixels[i * source->width + (j - 1)];
            }
            if (j < source->width - 1)
            {
                neighbors[count++] = source->pixels[i * source->width + (j + 1)];
            }

            // Arrays to hold the red, green, and blue values of the neighbors
            uint16_t red_values[5];
            uint16_t green_values[5];
            uint16_t blue_values[5];

            // Add the center pixel value to the arrays
            red_values[0] = source->pixels[i * source->width + j].red;
            green_values[0] = source->pixels[i * source->width + j].green;
            blue_values[0] = source->pixels[i * source->width + j].blue;

            // Add the neighboring pixel values to the arrays
            for (int i = 1; i <= count; i++)
            {
                red_values[i] = neighbors[i - 1].red;
                green_values[i] = neighbors[i - 1].green;
                blue_values[i] = neighbors[i - 1].blue;
            }

            // Sort the arrays
            qsort(red_values, count + 1, sizeof(uint16_t), compare);
            qsort(green_values, count + 1, sizeof(uint16_t), compare);
            qsort(blue_values, count + 1, sizeof(uint16_t), compare);

            // Apply the median filter into twop situation
            if (count < 4)
            {
                // apply the median filter to the border situation
                result->pixels[i * source->width + j].red = calculate_average(red_values, count + 1);
                result->pixels[i * source->width + j].green = calculate_average(green_values, count + 1);
                result->pixels[i * source->width + j].blue = calculate_average(blue_values, count + 1);
            }
            else
            {
                // apply the median filter to the normal situation
                result->pixels[i * source->width + j].red = red_values[count / 2 + 1];
                result->pixels[i * source->width + j].green = green_values[count / 2 + 1];
                result->pixels[i * source->width + j].blue = blue_values[count / 2 + 1];
            }
        }
    }

    return result;
}

/* Perform your second task.
 * (TODO: Write a better comment here, and rename the function.
 * You may need to add or change arguments depending on the task.)
 * Returns true on success, or false on error. */
bool apply_HIST(const struct Image *source)
{
    // Check if the input image is NULL
    if (source == NULL)
    {
        fprintf(stderr, "No source to be processed\n");
        return false;
    }

    // Get the width and height of the image
    int width = source->width;
    int height = source->height;

    // Declare and initialize the histogram array
    int hist[65536] = {0};

    // Calculate the histogram of the image
    for (int i = 0; i < width * height; i++)
    {
        // Increment the counts for the red, green, and blue channels separately
        hist[source->pixels[i].red]++;
        hist[source->pixels[i].green]++;
        hist[source->pixels[i].blue]++;
    }

    // Output the histogram statistics
    for (int i = 0; i < 65536; i++)
    {
        printf("Value %d: %d pixels\n", i, hist[i]);
    }

    return true;
}

int main(int argc, char *argv[])
{
    /* Check command-line arguments */
    if (argc % 2 != 1)
    {
        fprintf(stderr, "Usage: process INPUTFILE OUTPUTFILE\n");
        return 1;
    }

    LinkList *head = NULL;
    LinkList *tail = NULL;

    /* Load the input image */
    for (int i = 1; i < argc; i += 2)
    {
        Image *in_img = load_image(argv[i]);
        in_img->name = (argv[i + 1]);

        if (in_img == NULL)
        {
            free_image(in_img);
            return 1;
        }

        /* Create a new node for the linked list and check the malloc*/
        LinkList *new_LinkList = malloc(sizeof(LinkList));
        if (new_LinkList == NULL)
        {
            printf("Memory allocation failed.\n");
            return 1;
        }

        /* Initialize the new node */
        new_LinkList->image = in_img;
        new_LinkList->next = NULL;

        /* Append the new node to the linked list */
        if (!head)
        {
            head = new_LinkList;
            tail = new_LinkList;
        }
        else
        {
            tail->next = new_LinkList;
            tail = new_LinkList;
        }
    }

    /* Process each image in the linked list */
    LinkList *current = head;

    while (current)
    {
        // set flag for checking whether should return 1 when goto out
        int flag = 0;

        /* Apply median filter */
        struct Image *out_img = apply_MEDIAN(current->image);
        if (!apply_HIST(out_img))
        {
            fprintf(stderr, "First process failed.\n");
            flag = 1;
            goto out;
        }

        // Assign output image name
        out_img->name = current->image->name;

        if (out_img == NULL)
        {
            fprintf(stderr, "Second process failed.\n");
            flag = 1;
            goto out;
        }

        /* Save the output image */
        if (!save_image(out_img))
        {
            fprintf(stderr, "Saving image failed.\n");
            flag = 1;
            goto out;
        }

    out:
        /* Free memory for input and output images */
        free_image(current->image);
        free_image(out_img);
        if (flag)
        {
            flag = 0;
            return 1;
        }

        // Move to the next node
        current = current->next;
    }

    return 0;
}
