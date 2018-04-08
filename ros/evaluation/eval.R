wd="/data/ownCloud/Uni/Master/PG/pg2016w/ros/evaluation/ZMQ/results/"

#setwd("/data/ownCloud/Uni/Master/PG/pg2016w/ros/evaluation")

plot_cpu_load <- function(name) {
  #setwd(paste("/data/ownCloud/Uni/Master/PG/pg2016w/ros/evaluation/", name, sep=""))
  filepath=paste(wd,name,sep="")
  cpu_load_data <- read.csv(file=paste(filepath,"cpuload.csv",sep="/"),
                            header=TRUE, 
                            sep=",")
  
  timestamps <- as.numeric(cpu_load_data$Time) / 1000
  first_timestamp <- timestamps[1]
  timestamps <- timestamps - first_timestamp
  
  cpus <- data.frame(cpu_load_data$CPU1, 
                     cpu_load_data$CPU2, 
                     cpu_load_data$CPU3, 
                     cpu_load_data$CPU4)
  
  xrange <- range(timestamps)
  yrange <- range(0:100)
  
  n <- 4
  
  plot(xrange, yrange, 
       type="n", 
       xlab="Time (s)", ylab="COU load (%)" )
  linetype <- c(1:n)
  plotchar <- seq(18,18+n,1)
  colors <- rainbow(n) 
  
  lines(timestamps, cpus$cpu_load_data.CPU1, type="b", lwd=1.5, lty=linetype[1], col=colors[1], pch=plotchar[1])
  lines(timestamps, cpus$cpu_load_data.CPU2, type="b", lwd=1.5, lty=linetype[2], col=colors[2], pch=plotchar[2])
  lines(timestamps, cpus$cpu_load_data.CPU3, type="b", lwd=1.5, lty=linetype[3], col=colors[3], pch=plotchar[3])
  lines(timestamps, cpus$cpu_load_data.CPU4, type="b", lwd=1.5, lty=linetype[4], col=colors[4], pch=plotchar[4])
  
  legend(xrange[1], yrange[2], 1:n, cex=0.8, col=colors, pch=plotchar, lty=linetype, title="CPU")
  
}

read_files <- function(name) {
  setwd(paste(wd, name, sep=""))
  
  files = list.files(pattern="*.csv")[-1]
  
  file_data = lapply(files, read.csv, header=TRUE, 
                     sep=",",
                     colClasses=c("NULL", "NULL", "NULL", "NULL", "numeric"))
  
  min_length = nrow(data.frame(file_data[1]))
  
  if (length(file_data) > 1)
  {
    for (i in 2:length(file_data)) {
      min_length = min(min_length,nrow(data.frame(file_data[i])))
    }
  }
  
  frames <-data.frame(row.names=1:min_length)
  
  for (i in 1:length(file_data)) {
    df <- data.frame(file_data[i])$Duration[1:min_length]
    df <- df * 1000
    frames <- cbind(frames, df)
  }
  
  return(frames)
}

plot_delays <- function(name, plotnames) {
  #name="1_talkers_1_listeners"
  #plotnames=c(1)
  
  frames <- read_files(name)
  
  medians <- c()
  for (i in 1:length(frames)) {
    medians[i] <- round(median(frames[[i]]), 2)
  }
  medians
  colnames(frames) <- plotnames
  
  title=paste("n =",length((frames[[1]])))
  boxplot(frames, 
          main=title,
          las=2, 
          range=0.5,
          outline=F, 
          ylab="Time (ms)",
          par(mar = c(8, 5, 4, 2)+ 0.1))
  text(x = c(1:length(frames)), y = medians+0.06, labels = medians)
  
  
}

print_details <- function(name, plotnames) {
  frames <- read_files(name)
  
  for(i in 1:length(frames)) {
    frame <- frames[[i]]
    print(paste("Summary for", plotnames[[i]]))
    print(summary(frame))
    print(paste("Standard deviation:",sd(frame)))
    print(paste("Variance: ",var(frame),sep=""))
    print("----------------")
  }
}

# --- 1 Talker 1 Listener, Rate: 50Hz, Size: 1MB ---
# -- CPU load --
plot_cpu_load("1_talkers_1_listeners")
# -- Delay measurement results --
columns <- "50Hz 1MB"
plot_delays("1_talkers_1_listeners", columns)
print_details("1_talkers_1_listeners", columns)

# --- 5 Talkers each 2 Listeners, Rate: 100Hz, Size: 50kB ---
# -- CPU load --
plot_cpu_load("5_talkers_2_listeners")
# -- Delay measurement results --
columns <- c("100Hz 50kB", "100Hz 50kB",
             "100Hz 50kB", "100Hz 50kB",
             "100Hz 50kB", "100Hz 50kB",
             "100Hz 50kB", "100Hz 50kB",
             "100Hz 50kB", "100Hz 50kB")
plot_delays("5_talkers_2_listeners", columns)
print_details("5_talkers_2_listeners", columns)

# --- Realistic Test run
# -- CPU load --
plot_cpu_load("realistic_conditions")
# -- Delay measurement results --
columns <- c("50Hz 1MB",
             "100Hz 1kB","100Hz 1kB",
             "100Hz 1kB")
plot_delays("realistic_conditions", columns)
print_details("realistic_conditions", columns)