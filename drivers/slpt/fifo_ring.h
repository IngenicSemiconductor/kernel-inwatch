#ifndef _FIFO_RING_H_
#define _FIFO_RING_H_

#include <linux/slab.h>

struct time_vol_pair {
	unsigned int time;
	unsigned int voltage;
};

struct fifo_ring {
	struct time_vol_pair *data;
	unsigned int cur;
	unsigned int len;
	unsigned int top;
	unsigned int size;
};

static inline void fifo_ring_init(struct fifo_ring *fifo, struct time_vol_pair *data, unsigned int size) {
	fifo->data = data;
	fifo->size = size;
	fifo->cur = fifo->top = 0;
	fifo->len = 0;
}

static inline int fifo_ring_is_empty(struct fifo_ring *fifo) {
	return !fifo->len;
}

static inline int fifo_ring_size(struct fifo_ring *fifo) {
	return fifo->size;
}

static inline int fifo_ring_len(struct fifo_ring *fifo) {
	return fifo->len;
}

static inline void fifo_ring_add(struct fifo_ring *fifo, struct time_vol_pair value) {
	fifo->data[fifo->top] = value;
	if (++fifo->top == fifo->size)
		fifo->top = 0;
	if (++fifo->len > fifo->size) {
		fifo->len = fifo->size;
		if (++fifo->cur == fifo->size)
			fifo->cur = 0;
	}
}

static inline int fifo_ring_get(struct fifo_ring *fifo, struct time_vol_pair *valuep) {
	if (!fifo_ring_is_empty(fifo)) {
		*valuep = fifo->data[fifo->cur];
		if (++fifo->cur == fifo->size)
			fifo->cur = 0;
		fifo->len--;
		return 1;
	}
	return 0;
}

static inline unsigned int fifo_ring_top(struct fifo_ring *fifo) {
	unsigned int top = fifo->cur + fifo->len - 1;

	return (top >= fifo->size) ? (top - fifo->size) : top;
}

static inline int fifo_ring_get_reverse(struct fifo_ring *fifo, struct time_vol_pair *valuep) {
	unsigned int top;
	if (!fifo_ring_is_empty(fifo)) {
		top = fifo_ring_top(fifo);
		*valuep = fifo->data[top];
		fifo->len--;
		return 1;
	}
	return 0;
}

/* utils */
static inline struct fifo_ring *alloc_fifo_ring(unsigned int size) {
	struct fifo_ring *fifo;
	struct time_vol_pair *data;
	
	fifo = kmalloc(sizeof(*fifo), GFP_KERNEL);
	if (!fifo)
		return NULL;
	
	data = kmalloc(sizeof(*data) * size, GFP_KERNEL);
	if (!data) {
		kfree(fifo);
		return NULL;
	}

	fifo_ring_init(fifo, data, size);
	return fifo;
}

static inline void free_fifo_ring(struct fifo_ring *fifo) {
	kfree(fifo->data);
	kfree(fifo);
}

#endif /* _FIFO_RING_H_ */
