# Naming Convention

## Types

- `PascalCase`: The name starts with a capital letter, and has a capital letter for each new word, with no underscores.

- `camelCased`: Like CamelCase, but with a lower-case first letter

- `under_scored`: The name uses only lower-case letters, with words separated by underscores. (yes, I realize that under_scored should be underscored, because it's just one word).

- `ALL_CAPITALS`: All capital letters, with words separated by underscores.

## Conventions

|      Which       |       Type       | Example                       |
| :--------------: | :--------------: | :---------------------------- |
|     Package      |  `under_scored`  | my_pkg                        |
|      Topics      |  `under_scored`  | my_topic                      |
|      Files       |  `under_scored`  | my_src.cpp, my_head.h         |
|    Constants     |  `ALL_CAPITALS`  | const int MY_NUMBER = 0;      |
|    Variables     |  `under_scored`  | int my_var = 0;               |
| Global variables | `g_under_scored` | int g_my_member = 0;          |
|     Function     |   `camelCased`   | void myFunction(int an_arg){} |
|    Namespaces    |  `under_scored`  | my_namespace{}                |
|     Classes      |   `PascalCase`   | MyClass{}                     |
|     Methods      |   `camelCased`   | void myMethod(int an_arg){}   |
| Member variables |  `under_scored`  | int my_member = 0;            |
