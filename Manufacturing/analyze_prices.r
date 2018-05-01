library(Rcpp)

args <- commandArgs(TRUE)

file <- args[1]

if (length(args) < 1) {
    cat("Usage: ... <csvPriceFile>\n");
    quit(status=1)
}

unitsCol <- "units"
priceCol <- "price"
nbRobots <- 3

data <- read.csv(file)

print(data)

mec_entries <- which(data[,priceCol] < 200)
motor_entries <- which(data[priceCol] > 200)

price_mec_per_robot   <- sum(data[mec_entries, priceCol] * data[mec_entries, unitsCol])
price_motor_per_robot <- sum(data[motor_entries, priceCol] * data[motor_entries, unitsCol])

#print(data[mec_entries, priceCol])
#print(data[mec_entries, unitsCol])
print(paste("prix meca par robot:", price_mec_per_robot))
print(paste("prix moteurs par robot:", price_motor_per_robot))
print(paste("prix total par robot:", price_mec_per_robot + price_motor_per_robot))

data$nbUnits = nbRobots * data[,unitsCol] + data$upgrade -data$stock

negativeUnitsIdx = which(data$nbUnits < 0)

print(negativeUnitsIdx)

data[negativeUnitsIdx, "nbUnits"] = 0

# Adjust nbUnits with package
HN05I101Idx = which(data$part == "HN05-I101")
FR05F101Idx = which(data$part == "FR05-F101")
FR05S101Idx = which(data$part == "FR05-S101")
FR05H101Idx = which(data$part == "FR05-H101")
data[FR05S101Idx, "nbUnits"] = data[FR05S101Idx, "nbUnits"] - data[FR05F101Idx, "nbUnits"]
data[HN05I101Idx, "nbUnits"] = data[HN05I101Idx, "nbUnits"] - data[FR05F101Idx, "nbUnits"]
data[HN05I101Idx, "nbUnits"] = data[HN05I101Idx, "nbUnits"] - data[FR05H101Idx, "nbUnits"]

negativeUnitsIdx = which(data$nbUnits < 0)

print(negativeUnitsIdx)

data[negativeUnitsIdx, "nbUnits"] = 0

print(data$nbUnits)

data$totalPrice = data[,priceCol] * data$nbUnits 

totalPrice = sum(data$totalPrice)

print(totalPrice)

write.csv(data, file="postProcessed.csv")
