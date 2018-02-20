library(Rcpp)

args <- commandArgs(TRUE)

file <- args[1]

if (length(args) < 1) {
    cat("Usage: ... <csvPriceFile>\n");
    quit(status=1)
}

data <- read.csv(file)

mec_entries <- which(data$prix < 200)
motor_entries <- which(data$prix > 200)

price_mec_per_robot   <- sum(data[mec_entries, "prix"] * data[mec_entries, "parRobot"])
price_motor_per_robot <- sum(data[motor_entries, "prix"] * data[motor_entries, "parRobot"])

print(data[mec_entries, "prix"])
print(data[mec_entries, "parRobot"])
print(price_mec_per_robot)
print(price_motor_per_robot)

data$nbUnits = data$parRobot + data$upgradeNuc + data$marge -data$stock

negativeUnitsIdx = which(data$nbUnits < 0)
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
data[negativeUnitsIdx, "nbUnits"] = 0

print(data$nbUnits)

data$totalPrice = data$prix * data$nbUnits 

totalPrice = sum(data$totalPrice)

print(totalPrice)

write.csv(data, file="postProcessed.csv")
