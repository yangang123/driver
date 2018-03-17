main: main.o work_queue.o time.o
	gcc -o main main.o work_queue.o time.o

main.o: main.c time.h work_queue.h
	gcc -c main.c  

work_queue.o: work_queue.c time.h work_queue.h
	gcc -c work_queue.c

time.o: time.c time.h
	gcc -c time.c  

clean:
	rm -f main *.o
