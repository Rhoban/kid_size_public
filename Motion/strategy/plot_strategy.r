library(ggplot2)

args <- commandArgs(TRUE)

if (length(args) != 1) {
    print(paste("Usage: ... <strategy.csv>"))
    quit()
}

data <- read.csv(args[1])

print(names(data))

arrowLength <- 0.2
arrowWidth <- unit(0.2,"cm")

data$xend <- data$x + cos(data$kickDir) * arrowLength
data$yend <- data$y + sin(data$kickDir) * arrowLength

print(nrow(data))

g <- ggplot(data, aes(x=x,y=y,xend=xend,yend=yend,color=kickName))
g <- g + geom_segment(arrow=arrow(length=arrowWidth))
g <- g + coord_cartesian(xlim=c(0,9),ylim=c(0,6))
ggsave("test.png")
