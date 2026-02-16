# This macro converts the path to unix style for use in msys2 shells.
# Arguments are variable name and path

# UNIX_PATH: args = `variable`, `path`
macro(UNIX_PATH variable path)
    # escape backslashes
    string(REPLACE "\\" "/" temp_path "${path}")
    string(REPLACE " " "\\ " temp_path "${temp_path}")
    string(REPLACE "(" "\\(" temp_path "${temp_path}")
    string(REPLACE ")" "\\)" temp_path "${temp_path}")

    # extract and convert the drive letter to lowercase
    string(REGEX REPLACE "^([A-Za-z]):.*" "\\1" drive_letter "${temp_path}")
    string(TOLOWER "${drive_letter}" drive_letter)

    # replace the drive letter in the path with /<lowercase_letter>
    string(REGEX REPLACE "^[A-Za-z]:" "/${drive_letter}" temp_path "${temp_path}")

    # set the output variable
    set(${variable} "${temp_path}")
endmacro()
