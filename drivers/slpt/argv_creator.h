#ifndef _ARGV_CREATOR_H_
#define _ARGV_CREATOR_H_

struct argv_creator {
	/* public, valid on called alloc_argv_creator() */
	char **argv;				/* the argv */
	unsigned int argc;			/* the argc */

	/* private */
	char *buffer;
	char **save_ep;
	char *save_ec;
};

extern struct argv_creator *alloc_argv_creator(char *buffer);
extern void argv_creator_free(struct argv_creator *ac);

#endif /* _ARGV_CREATOR_H_ */
