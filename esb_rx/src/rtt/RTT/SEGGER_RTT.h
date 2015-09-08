/*********************************************************************
*               SEGGER MICROCONTROLLER GmbH & Co. KG                 *
*       Solutions for real time microcontroller applications         *
**********************************************************************
*                                                                    *
*       (c) 2014 - 2015  SEGGER Microcontroller GmbH & Co. KG        *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER RTT * Real Time Transfer form embedded targets        *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* * This software may in its unmodified form be freely redistributed *
*   in source form.                                                  *
* * The source code may be modified, provided the source code        *
*   retains the above copyright notice, this list of conditions and  *
*   the following disclaimer.                                        *
* * Modified versions of this software in source or linkable form    *
*   may not be distributed without prior consent of SEGGER.          *
* * This software may only be used for communication with SEGGER     *
*   J-Link debug probes.                                             *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************
*                                                                    *
*       RTT version: V2.10                                           *
*                                                                    *
**********************************************************************
---------------------------END-OF-HEADER------------------------------
File    : SEGGER_RTT.h
Purpose : Implementation of SEGGER real-time transfer which allows
          real-time communication on targets which support debugger 
          memory accesses while the CPU is running.
----------------------------------------------------------------------
*/

#ifndef SEGGER_RTT_H
#define SEGGER_RTT_H

/*********************************************************************
*
*       Defines, fixed
*
**********************************************************************
*/
#define SEGGER_RTT_VERSION    21000   // Format: Mmmrr. Example: 21000 is 2.10

/*********************************************************************
*
*       RTT API functions
*
**********************************************************************
*/

int          SEGGER_RTT_ConfigUpBuffer   (unsigned BufferIndex, const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags);
int          SEGGER_RTT_ConfigDownBuffer (unsigned BufferIndex, const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags);
int          SEGGER_RTT_GetKey           (void);
unsigned     SEGGER_RTT_HasData          (unsigned BufferIndex);
int          SEGGER_RTT_HasKey           (void);
void         SEGGER_RTT_Init             (void);
unsigned     SEGGER_RTT_Read             (unsigned BufferIndex,       void* pBuffer, unsigned BufferSize);
unsigned     SEGGER_RTT_ReadNoLock       (unsigned BufferIndex,       void* pData,   unsigned BufferSize);
int          SEGGER_RTT_SetNameDownBuffer(unsigned BufferIndex, const char* sName);
int          SEGGER_RTT_SetNameUpBuffer  (unsigned BufferIndex, const char* sName);
int          SEGGER_RTT_WaitKey          (void);
unsigned     SEGGER_RTT_Write            (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned     SEGGER_RTT_WriteNoLock      (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned     SEGGER_RTT_WriteSkipNoLock  (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned     SEGGER_RTT_WriteString      (unsigned BufferIndex, const char* s);


/*********************************************************************
*
*       RTT "Terminal" API functions
*
**********************************************************************
*/
void    SEGGER_RTT_SetTerminal        (char TerminalId);
int     SEGGER_RTT_TerminalOut        (char TerminalId, const char* s);

/*********************************************************************
*
*       RTT printf functions (require SEGGER_RTT_printf.c)
*
**********************************************************************
*/
int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/

//
// Operating modes. Define behavior if buffer is full (not enough space for entire message)
//
#define SEGGER_RTT_MODE_NO_BLOCK_SKIP         (0U)     // Skip. Do not block, output nothing. (Default)
#define SEGGER_RTT_MODE_NO_BLOCK_TRIM         (1U)     // Trim: Do not block, output as much as fits.
#define SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL    (2U)     // Block: Wait until there is space in the buffer.
#define SEGGER_RTT_MODE_MASK                  (3U)

//
// Control sequences, based on ANSI.
// Can be used to control color, and clear the screen
//
#define RTT_CTRL_RESET                "\e[0m"         // Reset to default colors
#define RTT_CTRL_CLEAR                "\e[2J"         // Clear screen, reposition cursor to top left

#define RTT_CTRL_TEXT_BLACK           "\e[2;30m"
#define RTT_CTRL_TEXT_RED             "\e[2;31m"
#define RTT_CTRL_TEXT_GREEN           "\e[2;32m"
#define RTT_CTRL_TEXT_YELLOW          "\e[2;33m"
#define RTT_CTRL_TEXT_BLUE            "\e[2;34m"
#define RTT_CTRL_TEXT_MAGENTA         "\e[2;35m"
#define RTT_CTRL_TEXT_CYAN            "\e[2;36m"
#define RTT_CTRL_TEXT_WHITE           "\e[2;37m"

#define RTT_CTRL_TEXT_BRIGHT_BLACK    "\e[1;30m"
#define RTT_CTRL_TEXT_BRIGHT_RED      "\e[1;31m"
#define RTT_CTRL_TEXT_BRIGHT_GREEN    "\e[1;32m"
#define RTT_CTRL_TEXT_BRIGHT_YELLOW   "\e[1;33m"
#define RTT_CTRL_TEXT_BRIGHT_BLUE     "\e[1;34m"
#define RTT_CTRL_TEXT_BRIGHT_MAGENTA  "\e[1;35m"
#define RTT_CTRL_TEXT_BRIGHT_CYAN     "\e[1;36m"
#define RTT_CTRL_TEXT_BRIGHT_WHITE    "\e[1;37m"

#define RTT_CTRL_BG_BLACK             "\e[24;40m"
#define RTT_CTRL_BG_RED               "\e[24;41m"
#define RTT_CTRL_BG_GREEN             "\e[24;42m"
#define RTT_CTRL_BG_YELLOW            "\e[24;43m"
#define RTT_CTRL_BG_BLUE              "\e[24;44m"
#define RTT_CTRL_BG_MAGENTA           "\e[24;45m"
#define RTT_CTRL_BG_CYAN              "\e[24;46m"
#define RTT_CTRL_BG_WHITE             "\e[24;47m"

#define RTT_CTRL_BG_BRIGHT_BLACK      "\e[4;40m"
#define RTT_CTRL_BG_BRIGHT_RED        "\e[4;41m"
#define RTT_CTRL_BG_BRIGHT_GREEN      "\e[4;42m"
#define RTT_CTRL_BG_BRIGHT_YELLOW     "\e[4;43m"
#define RTT_CTRL_BG_BRIGHT_BLUE       "\e[4;44m"
#define RTT_CTRL_BG_BRIGHT_MAGENTA    "\e[4;45m"
#define RTT_CTRL_BG_BRIGHT_CYAN       "\e[4;46m"
#define RTT_CTRL_BG_BRIGHT_WHITE      "\e[4;47m"


#endif

/*************************** End of file ****************************/
