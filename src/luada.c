// Todo: Support Windows or mingw.

#include <assert.h>             // for assert()
#include <dirent.h>             // for readdir()
#include <errno.h>              // for errno
#include <fcntl.h>              // for open(), O_RDWR, O_NOCTTY
#include <poll.h>               // for poll()
#include <readline/readline.h>  // for readline()
#include <readline/history.h>   // for add_history()
#include <stdbool.h>            // for bool
#include <stdlib.h>             // for free()
#include <string.h>             // for strncmp(), strcmp()
#include <termios.h>            // for tcsetattr() and tcgetattr()
#include <unistd.h>             // for STDIN_FILENO, isatty()

// Override the lua_writestringerror() definition in lauxlib.h.
#define lua_writestringerror(format, ...) \
    fprintf(stderr, format, __VA_ARGS__)

#include "lprefix.h"
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"



static const char DEVICE_DIR[] = "/dev/";
static const char TTYACM[] = "ttyACM";

static const char LUA_PROMPT[] = "> ";
static const char LUA_PROMPT2[] = "+ ";

// Additional Lua statuses
#define LUA_ERRIO   (-1)  // IO error (e.g. serial connection lost or EOF in stdin)
#define LUA_ERROPT  (-2)  // Command-line argument error

typedef int status_t;  // LUA_OK, LUA_ERRSYNTAX, etc

// Compile-time strlen(s) when s points to a literal string.
#define LEN(s)  (sizeof(s)/sizeof((s)[0]) - 1)

// Program name for this application, i.e. "luada".
static char* PROGNAME;  // Will be assigned with argv[0].

// Print an error message in printf() fashion, implicitly adding the program name at the
// beginning and a newline at the end. The format doesn't need to be a literal string if
// it is the only argument.
#define l_message(format, ...) \
    _l_message_ ## __VA_OPT__(1) (format __VA_OPT__(,) __VA_ARGS__)

#define _l_message_(s)  fprintf(stderr, "%s: %s\n", PROGNAME, (s))
#define _l_message_1(format, ...) \
    fprintf(stderr, "%s: " format "\n", PROGNAME, __VA_ARGS__)



// fds[0].fd will be set to the FD of the serial port.
#define serial_fd  (fds[0].fd)

static struct pollfd fds[] = {
    { -1, POLLIN, 0 },
    { STDIN_FILENO, POLLIN, 0 }
};

#define MAX_SERIAL_INPUT 1024
static char serial_buffer[MAX_SERIAL_INPUT];

// Try to receive Lua status remotely, which also indicates the remote Lua REPL is ready.
// Return the received status if successful, or LUA_ERRIO if it times out or if there is
// an error in reading from the serial port (e.g. serial connection lost).
// Using -1 for timeout_ms will wait indefinitely.
static status_t receive_status(bool recount, int timeout_ms)
{
    for (;;) {
        // Wait for an input from the serial port.
        int result = poll(fds, 1, timeout_ms);
        assert( result >= 0 );

        // Return false if it times out or if there is an error in the serial port.
        if ( result == 0 || (fds[0].revents & POLLERR) )
            return LUA_ERRIO;

        // As in cananical mode, read() receives a complete line that ends with '\n'.
        result = read(fds[0].fd, serial_buffer, sizeof(serial_buffer));
        assert( result >= 0 );

        // Got the 4 byte response "[}?\n". Extract the status at "?".
        if ( result == 4 && *(uint16_t*)serial_buffer == '[' + ('}' << 8) )
            return serial_buffer[2] - '0';

        // Print out any other messages from the serial port while waiting.
        if ( recount )
            lua_writestring(serial_buffer, result);
    }
}

static struct termios orig_serial_io_state;

// Return true if an appropriate "/dev/ttyACMx" is found, putting its fd in fds[0].fd.
static bool setup_serial(bool recount)
{
    DIR* pdir = opendir(DEVICE_DIR);
    if ( pdir == NULL ) {
        l_message("%s: %s", strerror(errno), DEVICE_DIR);
        return false;
    }

    struct dirent* pdirent = NULL;
    char device_path[sizeof(pdirent->d_name)/sizeof(char) + LEN(DEVICE_DIR)];
    strcpy(device_path, DEVICE_DIR);

    while ( (pdirent = readdir(pdir)) != NULL ) {
        if ( strncmp(pdirent->d_name, TTYACM, LEN(TTYACM)) != 0 )
            continue;

        strcpy(device_path + LEN(DEVICE_DIR), pdirent->d_name);
        fds[0].fd = open(device_path, O_RDWR | O_NOCTTY);

        if ( tcgetattr(fds[0].fd, &orig_serial_io_state) == 0 ) {
            // Set up the serial port for canonical input mode with all ECHOs disabled,
            // which should come before the first read(). In canonical input mode, input
            // is made available line by line. An input line is available when one of the
            // line delimiters is typed (NL, EOL, EOL2; or EOF at the start of line).
            // See Canonical Input Processing:
            //  https://tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html
            //  https://manual.cs50.io/3/cfmakeraw

            struct termios ios = orig_serial_io_state;
            // ios.c_cflag = B1152000 | CS8 | CLOCAL | CREAD;
            // ios.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable software flow control.
            ios.c_iflag = 0;  // Disable software flow control and spcial characters.
            ios.c_oflag = 0;  // raw output
            ios.c_lflag = ICANON;  // Enable canonical input mode and disable all ECHOs.
            tcsetattr(fds[0].fd, TCSANOW, &ios);

            if ( receive_status(recount, 100) == LUA_OK ) {  // 100 ms
                if ( recount )
                    printf("on %s\n", device_path);
                break;
            }

            // Restore the attributes if it is not our serial port.
            tcsetattr(fds[0].fd, TCSANOW, &orig_serial_io_state);
            close(fds[0].fd);
            fds[0].fd = -1;
        }
    }
    closedir(pdir);

    if ( pdirent == NULL ) {
        l_message("serial port not found");
        return false;
    }
    return true;
}

// Either read_error or read_line can be non-NULL, but not both.
static const char* read_error = NULL;
static char* read_line;

// Note that the readline library treats ^D as EOF only at the beginning of a line,
// otherwise deletes the character under cursor.
static void _cb_linehandler(char* line)
{
    // NULL line indicates EOF.
    if ( line == NULL )
        read_error = "Bye!";

    // If it is a blank line we stay in pushline() and read next line.
    else if ( rl_end )
        read_line = line;
}

// Read a line from stdin using the readline library. Push the line onto the Lua stack if
// successful, or an error message otherwise (e.g. EOF is hit). The line returned has the
// trailing newline removed, so only the text remains.
// ( -- line | )
static status_t pushline(lua_State* L, bool firstline)
{
    // We do not use the Lua global variable "_PROMPT" to show the prompt, as the
    // interpreter runs remotely.
    rl_callback_handler_install((firstline ? LUA_PROMPT : LUA_PROMPT2), _cb_linehandler);

    read_line = NULL;

    // Once read_error is set to true, we keep returning the same error.
    while ( read_error == NULL && read_line == NULL ) {
        // Wait for an input either from the serial port or from stdin.
        int result = poll(fds, 2, -1);  // -1 to wait indefinitely.
        assert( result >= 0 );

        if ( fds[0].revents & POLLERR ) {
            read_error = "serial connection lost";
            break;
        }

        // If a complete line is read from the serial port, show it on stdout.
        if ( rl_end == 0 && (fds[0].revents & POLLIN) ) {
            result = read(fds[0].fd, serial_buffer, sizeof(serial_buffer));
            assert( result >= 0 );

            rl_clear_visible_line();  // Erase the prompt.
            lua_writestring(serial_buffer, result);
            rl_reset_line_state();
            rl_redisplay();           // Show the prompt again.
        }

        if ( fds[1].revents & POLLIN )
            rl_callback_read_char();  // It will call _cb_linehandler() on a newline.
    }

    rl_clear_visible_line();  // Erase the prompt.
    rl_callback_handler_remove();

    if ( read_error ) {
        l_message(read_error);
        return LUA_ERRIO;  // Reading error
    }

    lua_pushstring(L, read_line);
    free(read_line);  // Free the memory allocated for read_line.
    return LUA_OK;
}

// Try to compile a line on top of the stack as "return <line>;". Push the the compiled
// chunk if successful, or a compile-error message.
// ( line -- line chunk | line error )
static status_t compile_expression(lua_State* L, const char* chunkname)
{
    const char* line = lua_tostring(L, -1);  // Original line.
    const char* s = lua_pushfstring(L, "return %s;", line);

    status_t status = luaL_loadbuffer(L, s, strlen(s), chunkname);
    lua_remove(L, -2);  // Remove the modified line.
    if ( status == LUA_OK ) {
        if ( line[0] != '\0' )  // Non-empty?
            add_history(line);  // Keep it in the readline history.
    }

    return status;
}

// Check if the given status indicates syntax error and the error message at the top of
// the stack ends with the "<eof>" mark for incomplete statements.
// ( x -- | x )
static bool incomplete(lua_State* L, status_t status)
{
    static const char EOFMARK[] = "<eof>";

    if ( status == LUA_ERRSYNTAX ) {
        size_t msg_l;
        const char* msg = lua_tolstring(L, -1, &msg_l);
        if ( msg_l >= LEN(EOFMARK)
          && strcmp(msg + msg_l - LEN(EOFMARK), EOFMARK) == 0 ) {
            lua_pop(L, 1);
            return true;
        }
    }
    return false;
}

// Try to compile the line on top of the stack as a statement, possibly combining more
// lines from stdin to make it a complete statement. Push the the compiled chunk if
// successful, or a compile-error message.
// ( line -- line chunk | line error | line )
static status_t compile_more_lines(lua_State* L)
{
    for (;;) {  // Repeat until gets a complete statement.
        size_t line_l;
        const char* line = lua_tolstring(L, -1, &line_l);

        // Try to compile the (combined) line as a statement.
        status_t status = luaL_loadbuffer(L, line, line_l, "=stdin");

        // If it compiled successfully, return ( -- line chunk ) with status = LUA_OK.
        // Otherwise, if it fails with no need for additional lines, return
        // ( -- line error ) with the compile-error as the status.
        if ( !incomplete(L, status) ) {
            add_history(line);  // Keep it in the readline history.
            return status;
        }

        // Read in an additional line. If it fails, return ( -- line ).
        status = pushline(L, false);
        if ( status != LUA_OK )
            return status;

        // ( line1 line2 -- line )
        lua_pushliteral(L, "\n");  // Add newline
        lua_insert(L, -2);         // between the two lines.
        lua_concat(L, 3);          // Join them.
    }
}

static status_t _writer(lua_State*, const void* pdata, size_t sz, void*)
{
    if ( write(fds[0].fd, pdata, sz) == (ssize_t)sz )
        return LUA_OK;
    else {
        l_message("cannot write to the serial port");
        return LUA_ERRIO;
    }
}

// If the status is LUA_OK, execute the chunk on the stack remotely and return the remote
// status. Otherwise, print the error message on the stack.
// ( chunk | error | -- )
static status_t dochunk(lua_State* L, status_t status)
{
    if ( status == LUA_OK ) {
        // In the case lua_dump() fails, it returns the non-zero status from _writer(),
        // not removing the chunk on the stack.
        status = lua_dump(L, _writer, NULL, 0);  // 0 == strip disabled
        lua_pop(L, 1);  // Pop the compiled chunk.
    }

    if ( status == LUA_OK ) {
        // For the status received remotely, there is no associated error message on the
        // stack. We only pass the received status.
        status = receive_status(true, -1);
    }
    else if ( status == LUA_ERRSYNTAX ) {
        // Do not prepend the program name to the error message, since the syntax error
        // message already contains the chunk name.
        lua_writestringerror("%s\n", lua_tostring(L, -1));
        lua_pop(L, 1);  // Pop the error message.
    }
    // LUA_ERRIO does not have the associated error message on the stack.
    else if ( status != LUA_ERRIO ) {
        l_message(lua_tostring(L, -1));
        lua_pop(L, 1);  // Pop the error message.
    }

    // Sanity check
    assert( lua_gettop(L) == 0 );
    return status;
}

// ( -- )
static status_t dostring(lua_State* L, const char* s)
{
    lua_pushstring(L, s);
    status_t status = compile_expression(L, "=script");
    if ( status != LUA_OK ) {
        lua_pop(L, 1);  // Pop the error message.
        status = luaL_loadbuffer(L, s, strlen(s), "=script");
    }

    lua_remove(L, -2);  // Remove the line.
    return dochunk(L, status);
}

// If filename is NULL, it loads from the stdin.
// ( -- )
static inline status_t dofile(lua_State* L, const char* filename)
{
    return dochunk(L, luaL_loadfile(L, filename));
}

// ( -- )
static status_t doREPL(lua_State* L)
{
    status_t status;
    do {
        assert( lua_gettop(L) == 0 );
        status = pushline(L, true);
        if ( status == LUA_OK ) {
            status = compile_expression(L, "=stdin");  // Try as "return ...".
            if ( status != LUA_OK ) {
                lua_pop(L, 1);  // Pop the error message.
                // Try as command, possibly reading in more lines from stdin.
                status = compile_more_lines(L);
            }
            lua_remove(L, 1);  // Remove the line from the stack.
        }
    } while ( dochunk(L, status) != LUA_ERRIO );

    return LUA_OK;  // Return LUA_OK always.
}



// We run lua_* functions in unprotected mode, since we are only using some stack
// manipulation functions, luaL_loadbuffer() and lua_dump(), which basically do not
// throw an exception other than not enough memory.
status_t main(int argc, char* argv[])
{
    if ( argv[0] )
        PROGNAME = argv[0];

    bool interactive = (argc <= 1);
    if ( !interactive && strcmp(argv[argc-1], "-") == 0 ) {
        argc--;
        interactive = true;
    }

    // Note that we don't check LUA_VERSION_NUM and LUAL_NUMSIZES against the device.
    // A bytecode contains those information as a header and the device will check them
    // remotely for every bytecode that it receives.
    if ( !setup_serial(interactive) )
        return LUA_ERRIO;

    lua_State* L = luaL_newstate();
    if ( L == NULL ) {
        l_message("cannot create state: not enough memory");
        return LUA_ERRMEM;
    }

    // We don't need any standard libraries, not even the base library (lbaselib).
    // luaL_openlibs(L);  // Open all standard libraries.

    // Simple command-line parser
    status_t status = LUA_OK;
    for ( int i = 1 ; status == LUA_OK && i < argc ; i++ ) {
        const char* arg = argv[i];
        if ( arg[0] == '-' )
            switch ( arg[1] ) {
            case 'f':
                if ( arg[2] != '\0' )
                    status = dofile(L, &arg[2]);
                else if ( ++i < argc )
                    status = dofile(L, argv[i]);
                else  // fall through

            default: {  // Handle option errors and print usage.
                    status = LUA_ERROPT;
                    l_message("invalid option: %.2s", arg);
                    printf(
                        "Usage: %s [option|'<lua_code>']... [-]\n"
                        "\n"
                        "  -f <script-file>\texecute Lua script file\n"
                        "  '<lua_code>'\t\texecute inline Lua code\n"
                        "  - (at the end)\texecute REPL finally\n",
                        PROGNAME );
                }
                break;
            }

        else
            status = dostring(L, arg);
    }

    if ( status == LUA_OK && interactive )
        // If stdin is not `tty` (i.e. redirecting from a file), process it as a file.
        status = isatty(STDIN_FILENO) ? doREPL(L) : dofile(L, NULL);

    lua_close(L);
    return status;
}
