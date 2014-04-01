function(){

	prevRound <- -1
	roundNumber <- 0

	#inf loop here
	repeat{
		index <- 0

		while( roundNumber != prevRound ){
			line <- scan("input.txt", nlines = 1, skip = index, sep = " ", quiet = TRUE)
			
			if(line[1] == -3 && line[2] == roundNumber + 1){
				
				
				#input has changed, get number of nodes
				index <- index + 1 
				line <- scan("input.txt", nlines = 1, skip = index, sep = " ", quiet = TRUE)
				size <- line[3] 
				
				#make empty matrix/names matrix
				data <- matrix(c(rep(NA,size*size)), size, size) 
				desired <- matrix(c(rep(NA,size*2)), size, 2)
				
				index <- index + 1
				repeat{
					line <- scan("input.txt", nlines = 1, skip = index, sep = " ", quiet = TRUE)
					#if EOF, stop scanning
					if(!length(line)) break 
					
					#special data handling
					if(line[1] == -1){
						#if special line, and second bit is less than the number of nodes, grab next line as a name
						if(line[2] == 1) x1 <- line[3] 
						if(line[2] == 2) y1 <- line[3] 
						if(line[2] == 3) x2 <- line[3] 
						if(line[2] == 4) y2 <- line[3] 
						if(line[2] == 5) x3 <- line[3] 
						if(line[2] == 6) y3 <- line[3] 
					}else if(line[1] == -2){
						#desired coordinates handling
						#convert back to -127 to 127
						desired[line[2] + 1, line[3] + 1] <- line[4] # - 128  #stub put this line back in once we have real input
						
						#the following commented out might be necessary, but I think the above line will work
						#if(line[4] >= 128){
						#	desired[line[2] + 1, line[3] + 1] <- line[4] - 128
						#}else if(line[4] < 128){
						#	desired[line[2] + 1, line[3] + 1] <- (128 - line[4]) *-1
						#}
						
						
					}else if(line[1] == -4){
						#do nothing, don't worry about this line
					}else{
						#normal input handling
						data[line[1] + 1, line[2] + 1] <- line[3]
					}
					
					index <- index + 1				
				}
				
				prevRound <- roundNumber
			}else{
				Sys.sleep(1);
			}
		}
		
		roundNumber <- roundNumber + 1;
		


		#Do MDS
		fit <- cmdscale(data, eig=TRUE, k=2)

		#translate by input a,b minus where calc'd coords of point A lie
		moveX <- x1 - fit$points[1,1]
		moveY <- y1 - fit$points[1,2]

		for(i in 1:nrow(fit$points)){
			fit$points[i, 1] <- fit$points[i,1] + moveX
			fit$points[i, 2] <- fit$points[i,2] + moveY
		}

		#find amount to rotate
		#first, angle of calc'd first two points
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


		#repeat for angle of known first two points
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
		
		#Testing for flippedness
		tol <- 4
		if((fit$points[3,1] > x3 + tol) || (fit$points[3,1] < x3 - tol) || (fit$points[3,2] > y3 + tol) || (fit$points[3,2] < y3 - tol)){
			
			#flip arbitrarily across X axis
			flip <- matrix(c(1,0,0,-1),2,2)
			fit$points <- fit$points %*% flip
			test <- fit$points
			
			#Repeat above processes
			#find amount to rotate
			#first, angle of calc'd first two points
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


			#repeat for angle of known first two points
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
		
		
		
		#turn 0-number of nodes into a, b, c, d...
		alphabet <- c("a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z") 
		names <- array(c(rep(NA, size))) 
		for(i in 1:size){
			names[i] <- alphabet[i]
		}
		
		xy <- c("x","y")
		colnames(desired) <- xy
		rownames(desired) <- names
		
		#colnames(data) <- temp 
		#rownames(data) <- names 
		
		colnames(fit$points) <- xy
		rownames(fit$points) <- names
		
		#dev.new()
		x <- fit$points[,1]
		y <- fit$points[,2]
		plot(x, y, xlab="Inches", ylab="Inches", main="Network Map", type="n",xlim=c(-100,100),ylim=c(-100,100),)
		text(x,y,labels = row.names(fit$points), cex=1.2)
		abline(h=c(0), lty=1)
		abline(v=c(0), lty=1)
		text(x + 15.0, y, paste("(", round(x), ", ", round(y), ")"), cex=0.7)
		#par(new=TRUE)
		
		xx <- desired[,1]
		yy <- desired[,2]
		text(xx,yy,labels = row.names(desired), cex=1.2)
		text(xx + 15.0, yy, paste("(", round(xx), ", ", round(yy), ")"), cex=0.7)
		
		arrows(x, y, xx, yy, length = .15, angle = 20, code = 2, col = "red")
		
		
		#print to file
		cat(roundNumber, file = "output.txt", append = FALSE, sep = "\n")
		for(i in 1:size){
			cat(round(x[i] + 126), file = "output.txt", append = TRUE)
			cat(" ", file = "output.txt", append = TRUE)
			cat(round(y[i] + 126), file = "output.txt", append = TRUE, sep = "\n")
		}
		cat(roundNumber, file = "output.txt", append = TRUE, sep = "\n")
		
		#write.table(fit$points, file = "output.txt", sep = " ")
		
		
		#print to jpeg
		#fileName <- toString("pic", roundNumber, ".jpeg");
		#jpeg(file = "test.jpeg");
		#dev.off()
		
		#dev.copy(jpeg, filename="plot.jpg")
		#dev.off()
	}
	
	return(fit)
}
