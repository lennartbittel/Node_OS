#include <stdio.h>
#include <stdlib.h>
#include <time.h>

struct Test {
    int data;
    int i;
};

int main(int argc, char **argv)
{
    srand(time(NULL));

    // allocate 100 pointers, effectively an array
    struct Test **t_array; 
	t_array= malloc(100 * sizeof(struct Test *));
	printf("%d\n",sizeof(struct Test *));
    // allocate 100 structs and have the array point to them
    for (int i = 0; i < 100; i++) {
        t_array[i] = malloc(sizeof(struct Test));
    }

    // lets fill each Test.data with a random number!
    for (int i = 0; i < 100; i++) {
        t_array[i]->data = rand() % 100;
    }

    // now define a pointer to the array
    struct Test ***p = &t_array;
    printf("p points to an array of pointers.\n"
       "The third element of the array points to a structure,\n"
       "and the data member of that structure is: %d\n", (*p)[2]->data);

    return 0;
}
