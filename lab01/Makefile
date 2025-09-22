CC      = gcc
CFLAGS  = -Wall -Wextra -g
OBJDIR  = .obj
NAME1   = read_noncanonical
NAME2   = write_noncanonical

SRC1    = read_noncanonical.c
SRC2    = write_noncanonical.c

OBJ1    = $(OBJDIR)/$(SRC1:.c=.o)
OBJ2    = $(OBJDIR)/$(SRC2:.c=.o)

all: $(NAME1) $(NAME2)

$(NAME1): $(OBJ1)
	$(CC) $(CFLAGS) -o $@ $^

$(NAME2): $(OBJ2)
	$(CC) $(CFLAGS) -o $@ $^

$(OBJDIR)/%.o: %.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR):
	mkdir -p $(OBJDIR)

clean:
	rm -rf $(OBJDIR)

fclean: clean
	rm -f $(NAME1) $(NAME2)

re: fclean all

.PHONY: all clean fclean re

