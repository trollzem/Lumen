# Source Code

Code should be documented using Doxygen syntax.

## Guidelines

### Doxygen Comments

* Use Doxygen comments to document all files, functions, classes, and variables.
* [Inline documenation block](#inline-documentation-blocks) should use the following format:

  @verbatim
///< A brief description of the member.
@endverbatim

* Multi-line comments, such as for a [documentation block](#documentation-blocks), should use the following format:

  @verbatim
/**
 * @brief A brief description of the member.
 * More detailed description of the member.
 */
@endverbatim

### Documentation Blocks

Documentation blocks should be placed above the declaration of the function, class, or variable. Below is an example
of a documentation block for the main function.

@verbatim
/**
 * @brief Main application entry point.
 * @param argc The number of arguments.
 * @param argv The arguments.
 * @return The exit code.
 * @examples
 * main(1, const char* args[] = {"hello", "markdown", nullptr});
 * @examples_end
 */
int main(int argc, char *argv[]);
@endverbatim

@note The \@examples and \@examples_end tags are not standard Doxygen tags. They are custom aliases
we have specified to simplify documenting examples. Do not confuse this with the standard \@example tag.

In some cases, it could be valuable to have slightly different documentation for the definitions, especially when
the definition may change depending on the platform or other factors. In such cases, you should put the documentation
that is common in the declaration and the definition-specific documentation in the definition. Below is an example of
how to document the declaration and definition of a function.

@verbatim
// myFile.h
/**
 * @brief A brief description of the function.
 * @param arg1 Describe the first argument.
 * @param arg2 Describe the second argument.
 * @return Describe the result.
 */
int myFunction(int arg1, int arg2);

// myFile.cpp
/**
 * This describes anything which is specific to the implementation of the function.
 */
int myFunction(int arg1, int arg2)
{
    // Implementation
}
@endverbatim

### File Documentation

The file documentation block must be placed at the top of the file. If it is not present, Doxygen will ignore the file.
Understandably, it is difficult to make a creative description for every file, although it is still required.
Below is an example of a file documentation block.

@verbatim
/**
 * @file src/main.cpp
 * @brief Main application entry point.
 */
@endverbatim

### Inline Documentation Blocks

Inline comments can be used to describe enum values, variables, and other code constructs.
To document the members of a file, struct, union, class, or enum, it is sometimes desired to place the documentation
block after the member instead of before. For this purpose you have to put an additional `<` marker in the comment
block. Below is an example of an inline comment for an enum value.

@verbatim
enum class MyEnum
{
   FIRST_VALUE,  ///< A brief description of the FIRST_VALUE
   SECOND_VALUE  ///< A brief description of the SECOND_VALUE
};
@endverbatim

### Admonitions

#### Standard Admonitions

* @verbatim@attention Attention needed.@endverbatim
  @attention Attention needed.

* @verbatim@important An important message.@endverbatim
  @important An important message.

* @verbatim@note A note.@endverbatim
  @note A note.

* @verbatim@pre A precondition.@endverbatim
  @pre A precondition.

* @verbatim@remark Some remarks.@endverbatim
  @remark Some remarks.

* @verbatim@warning - A warning.@endverbatim
  @warning A warning.

#### GitHub Flavored Markdown Admonitions

* @verbatim
> [!CAUTION]
> This is a caution.
@endverbatim
  > [!CAUTION]
  > This is a caution.

* @verbatim
> [!IMPORTANT]
> This is important.
@endverbatim
  > [!IMPORTANT]
  > This is important.

* @verbatim
> [!NOTE]
> This is a note.
@endverbatim
  > [!NOTE]
  > This is a note.

* @verbatim
> [!TIP]
> This is a tip.
@endverbatim
  > [!TIP]
  > This is a tip.

* @verbatim
> [!WARNING]
> This is a warning.
@endverbatim
  > [!WARNING]
  > This is a warning.

### Custom Aliases

We have defined some custom aliases to simplify documenting examples.

* \@examples - Start of an example block. This will format the following text as `cpp`.
* \@examples_end - End of an example block.
* \@fa_icon{font awsome definition} - Insert a Font Awesome icon. Give the icon style and name as arguments.
  e.g. \@fa_icon{fa-solid fa-check}.
* Admonitions
  * \@admonition{Custom Title | Content }
    @admonition{The one with the custom titles | It’s got a certain charm to it.}
  * \@attention{Content}
    @attention{The sun causes global warming.}
  * \@caution{Content}
    @caution{Cliff ahead: Don’t drive off it.}
  * \@danger{Content}
    @danger{Mad scientist at work!}
  * \@error{Content}
    @error{Does not compute.}
  * \@hint{Content}
    @hint{Insulators insulate, until they are subject to ______ voltage.}
  * \@important{Content}
    @important{Tech is not neutral, nor is it apolitical.}
  * \@note{Content}
    @note{This is a note.}
  * \@seealso{Content}
    @seealso{Other relevant information.}
  * \@tip{Content}
    @tip{25% if the service is good.}
  * \@todo{Content}
    @todo{Fix this.}
  * \@warning{Content}
    @warning{Reader discretion is strongly advised.}

* Markers
  * \@red{Content}<br>
    @red{This is red.}
  * \@blue{Content}<br>
    @blue{This is blue.}
  * \@green{Content}<br>
    @green{This is green.}
  * \@yellow{Content}<br>
    @yellow{This is yellow.}

* Expander
  @verbatim
  @expander{Expander Title | Expander Content}
  @endverbatim
  @expander{Expander Title | Expander Content}

* Tabs
  @verbatim
@tabs{
  @tab{ Tab 1 Title | Tab 1 Content }
  @tab{ Tab 2 Title | Tab 2 Content }
}
@endverbatim
  @tabs{
    @tab{ Tab 1 Title | Tab 1 Content }
    @tab{ Tab 2 Title | Tab 2 Content }
  }

<details style="display: none;">
  <summary></summary>
  [TOC]
</details>
