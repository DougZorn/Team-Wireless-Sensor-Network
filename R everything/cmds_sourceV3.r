function(){

	prevRound <- -1
	roundNumber <- 0

	#Infinite loop, CMDS will run until ESC is hit in R command window
	repeat{
		index <- 0
		
		#Loop to look at input file, only processes input if
		#round number changes in line 1
		while(roundNumber != prevRound ){
			line <- scan("input.txt", nlines = 1, skip = index, sep = " ", quiet = TRUE)
			
			#Check for good file
			if(!is.na(line[1])){
				print("...")
				flush.console()
				
				#If round has changed from previously read round
				if(line[1] == -3 && line[2] != prevRound){   # roundNumber + 1){
					
					#Input has changed, get number of nodes
					index <- index + 1 
					line <- scan("input.txt", nlines = 1, skip = index, sep = " ", quiet = TRUE)
					size <- line[3] 
					
					#Make empty matrix/names matrix of appropriate sizes
					data <- matrix(c(rep(0,size*size)), size, size) 
					desired <- matrix(c(rep(0,size*2)), size, 2)
					
					index <- index + 1
					
					#Everything is checked, so read entire file
					repeat{
						line <- scan("input.txt", nlines = 1, skip = index, sep = " ", quiet = TRUE)
						#if EOF, stop scanning
						if(!length(line)) break 
						
						#Special data handling
						if(line[1] == -1){
							#These are anchor node values
							if(line[2] == 1) x1 <- line[3] 
							if(line[2] == 2) y1 <- line[3] 
							if(line[2] == 3) x2 <- line[3] 
							if(line[2] == 4) y2 <- line[3] 
							if(line[2] == 5) x3 <- line[3] 
							if(line[2] == 6) y3 <- line[3] 
						}else if(line[1] == -2){
							#Desired coordinates handling
							#convert back to -127 to 127 stub
							desired[line[2] + 1, line[3] + 1] <- line[4] # - 128  #stub put this line back in once we have real input
							
							#the following commented out bit might be necessary, but I think the above line will work
							#if(line[4] >= 128){
							#	desired[line[2] + 1, line[3] + 1] <- line[4] - 128
							#}else if(line[4] < 128){
							#	desired[line[2] + 1, line[3] + 1] <- (128 - line[4]) *-1
							#}
							
						}else if(line[1] == -4){
							#do nothing, don't worry about this line
							#it is necessary for arduino reading back from Serial
						}else{
							#Node Data input handling
							data[line[1] + 1, line[2] + 1] <- line[3]
						}	
						index <- index + 1				
					}
					prevRound <- roundNumber
				}else{
					#If a new round hasn't been written to the file, wait a second
					Sys.sleep(1);
				}
			}
		}
		
		#stub change this to capture the number from line 1 of initial grab
		roundNumber <- roundNumber + 1;
		
		
		print("Round Change, processing..")
		flush.console()
		
		#Checking for any inappropriate null values
		hasNA <- FALSE;
		for(i in 1:size){
			for(j in 1:size){
				if(i != j){
					if(data[i,j] == 0) hasNA <- TRUE
				}
			}
		}
		
		#MDS will not run if there are null values in dissimilarity matrix ("NA" values)
		if(!hasNA){
			print("Has NA values still")
			flush.console()
		}else{
			#Do MDS
			fit <- cmdscale(data, eig=TRUE, k=2)
			
			#At least one eigenvalue must be nonzero for MDS to be a success
			if(any(fit$eig >0)){
				#Find how much to translate results to center around node A (command node)
				moveX <- x1 - fit$points[1,1]
				moveY <- y1 - fit$points[1,2]
				
				#..and shift all points by this amount
				for(i in 1:nrow(fit$points)){
					fit$points[i, 1] <- fit$points[i,1] + moveX
					fit$points[i, 2] <- fit$points[i,2] + moveY
				}

				#Find amount to rotate
				#First calculate angle between two known points A and B
				ax <- fit$points[1,1]
				ay <- fit$points[1,2]
				bx <- fit$points[2,1]
				by <- fit$points[2,2]

				#First calculation for degree of rotation that the calculated graph has
				q <- atan(abs( by - ay )/abs( bx - ax ))
				
				#With offset handling based on current quadrant B is in
				if(bx >= 0 && by > 0){
					#nothing needed
				}else if(bx < 0 && by >= 0){
					q = pi/2 - q + (pi/2)
				}else if(bx < 0 && by < 0){
					q = q + pi
				}else if(bx >= 0 && by < 0){
					q = pi/2 - q + (3*pi/2)
				}

				#Repeat to find degree of rotation for anchor nodes (the rotation we want)
				r <- atan(abs( y2 - y1 )/abs( x2 - x1 ))
				
				if(x2 >= 0 && y2 > 0){
					#nothing needed
				}else if(x2 < 0 && y2 >= 0){
					r = pi/2 - r + (pi/2)
				}else if(x2 < 0 && y2 < 0){
					r = r + pi
				}else if(x2 >= 0 && y2 < 0){
					r = pi/2 - r + (3*pi/2)
				}

				#Combine Angles to get amount to rotate graph
				if(r <= q) theta <- q - r
				else if(r > q) theta <- 2*pi - (r - q)

				#Make Rotation matrix, multiply to rotate graph
				rotate <- matrix(c(cos(theta), sin(theta), -sin(theta), cos(theta)), 2,2)
				
				fit$points <- fit$points %*% rotate
				
				
				#Test for flip.  Use third anchor point to test if matrix has flipped orientation
				#"tol" is tolerance of how close points need to be to deem a flip necessary
				tol <- 4
				if((fit$points[3,1] > x3 + tol) || (fit$points[3,1] < x3 - tol) || (fit$points[3,2] > y3 + tol) || (fit$points[3,2] < y3 - tol)){
					
					#Flip is needed, so flip across X axis
					flip <- matrix(c(1,0,0,-1),2,2)
					fit$points <- fit$points %*% flip
					test <- fit$points
					
					#Repeat above rotation processes
					#First rotation computation
					ax <- fit$points[1,1]
					ay <- fit$points[1,2]
					bx <- fit$points[2,1]
					by <- fit$points[2,2]

					q <- atan(abs( by - ay )/abs( bx - ax ))
					
					if(bx >= 0 && by > 0){
						#nothing needed
					}else if(bx < 0 && by >= 0){
						q = pi/2 - q + (pi/2)
					}else if(bx < 0 && by < 0){
						q = q + pi
					}else if(bx >= 0 && by < 0){
						q = pi/2 - q + (3*pi/2)
					}


					#Second rotation computation
					r <- atan(abs( y2 - y1 )/abs( x2 - x1 ))
					
					if(x2 >= 0 && y2 > 0){
						#nothing needed
					}else if(x2 < 0 && y2 >= 0){
						r = pi/2 - r + (pi/2)
					}else if(x2 < 0 && y2 < 0){
						r = r + pi
					}else if(x2 >= 0 && y2 < 0){
						r = pi/2 - r + (3*pi/2)
					}

					#Combine Angles to get rotational angle
					if(r <= q) theta <- q - r
					else if(r > q) theta <- 2*pi - (r - q)

					#Make Rotation matrix, multiply
					rotate <- matrix(c(cos(theta), sin(theta), -sin(theta), cos(theta)), 2,2)
					
					fit$points <- fit$points %*% rotate
				}
				
				#End of calculation, begin display
				
				#Name nodes
				alphabet <- c("a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z") 
				names <- array(c(rep(NA, size))) 
				for(i in 1:size){
					names[i] <- alphabet[i]
				}
				xy <- c("x","y")
				colnames(desired) <- xy
				rownames(desired) <- names
				
				colnames(fit$points) <- xy
				rownames(fit$points) <- names
				
				#dev.new()  #needed for some applications of this program
				#Plot calculated points
				x <- fit$points[,1]
				y <- fit$points[,2]
				plot(x, y, xlab="Inches", ylab="Inches", main="Network Map", type="n",xlim=c(-100,100),ylim=c(-100,100),)
				text(x,y,labels = row.names(fit$points), cex=1.2)
				abline(h=c(0), lty=1)
				abline(v=c(0), lty=1)
				text(x + 15.0, y, paste("(", round(x), ", ", round(y), ")"), cex=0.7)
				#par(new=TRUE)
				
				#Plot desired points
				xx <- desired[,1]
				yy <- desired[,2]
				text(xx,yy,labels = row.names(desired), cex=1.2)
				text(xx + 15.0, yy, paste("(", round(xx), ", ", round(yy), ")"), cex=0.7)
				
				#Draw a nifty arrow
				arrows(x, y, xx, yy, length = .15, angle = 20, code = 2, col = "red")
				
				
				#Round calculations, print to file
				cat(roundNumber, file = "output.txt", append = FALSE, sep = "\n")
				for(i in 1:size){
					cat(round(x[i] + 126), file = "output.txt", append = TRUE)
					cat(" ", file = "output.txt", append = TRUE)
					cat(round(y[i] + 126), file = "output.txt", append = TRUE, sep = "\n")
				}
				cat(roundNumber, file = "output.txt", append = TRUE, sep = "\n")
			
			} #End of eig check case
		} #End of NA check case
		
		#print to jpeg 
		#fileName <- toString("pic", roundNumber, ".jpeg");
		#jpeg(file = "test.jpeg");
		#dev.off()
		
		#dev.copy(jpeg, filename="plot.jpg")
		#dev.off()
	}
	return(fit)
}
