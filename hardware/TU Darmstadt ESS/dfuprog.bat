:try
SET /A tries=%1
  :loop
  IF %tries% LEQ 0 GOTO return
  SET /A tries-=1
  EVAL %2 && (GOTO return) || (GOTO loop)
  :return


SET DIRECTORY=%0%\..\
ECHO %DIRECTORY%

CALL :try 100 "%DIRECTORY%"\dfu-programmer.exe %1% erase

CALL "%DIRECTORY%"\dfu-programmer.exe %1% flash %2% %3% %4%
CALL "%DIRECTORY%"\dfu-programmer.exe %1% start

