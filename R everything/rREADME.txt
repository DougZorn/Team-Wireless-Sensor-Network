asfd test

Here's what you need to know to run R, with the function I wrote

download R from google drive or from
http://cran.rstudio.com/

Start up R x64 3.0.2
have a folder on your desktop called Rstuff, containing at least
input.txt
output.txt


copy the following (minus the comments) into the command line, hit enter after each


//(change "eswedber" to your username)
setwd("c:/Users/eswedber/Desktop/Rstuff");   

cmds <- function(){ };

cmds <- edit(cmds);

//at this point, open up the cmds_source file,
//and copy/paste it all into the box that opens,
// Ctrl + S, then close the box.  Nothing should complain.
// the next command carrot ">" should pop up in the main
// R window and nothing else

cmds();

//That should run the program


Further notes:

The program does everything on its own.  It looks for a file called
"input.txt" in the folder set by setwd.  The format should be like
what I describe in inputREADME.txt .

The program will run infinitely, until you click into the main R window
and hit ESC.

It waits in an infinite loop, looking at input.txt for the first line
to change the roundNumber variable to the next value.

ex.  "-3 0" as the first line changes to "-3 1"

At which point it will unblock that loop, parse the input, run MDS
fiddle with the graph until it gets it right, display the graph,
then output the results to output.txt in the form demonstrated
by the output.txt file in the google drive.