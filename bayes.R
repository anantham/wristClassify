library('caret')
library(plyr)
library(MESS)
library(lsr)
library(seewave)

#daliac_i = subset(dataset_i, select=c("V1", "V2", "V3", "V25"))

# Construct dataset dim(TotalData) = 4686842 obs with 3 predictors 
#TotalData <- rbind(daliac_1, daliac_2, daliac_3, daliac_4, daliac_5, daliac_6, daliac_7, daliac_8, daliac_9, daliac_10, daliac_11, daliac_12, daliac_13, daliac_14, daliac_15, daliac_16, daliac_17, daliac_18, daliac_19)

TotalData <- read.csv("F:/Projects/wristClassify/TotalData.txt", sep="")
# Accelerometer axis i = Ai
colnames(TotalData) <- c("A1","A2","A3","Label")
TotalData$Label <- as.factor(TotalData$Label)

# Sitting 1
# Lying 2
# Standing 3
# Washing dishes 4
# Vacuuming 5
# Sweeping 6
# Walking 7
# Ascending stairs 8
# Descending stairs 9
# Treadmill running 10
# Bicycling on ergometer (50W) 11
# Bicycling on ergometer (100W) 12
# Rope jumping 13


TotalData$Label <- revalue(TotalData$Label, c("1"="Stationary", "2"="Stationary", "3"="Stationary", "4"="rm", "5"="rm", "6"="rm", "7"="Walking", "8"="Walking", "9"="Walking", "10"="Running", "11"="rm", "12"="rm", "13"="rm"))
#TotalData <- TotalData[!(TotalData$Label == "rm"),]

summary(TotalData)
#write.table(TotalData,file="TotalData.txt",quote=F,row.names = F,col.names = T)



Stationary = TotalData[(TotalData$Label == "Stationary"),]
Walking = TotalData[(TotalData$Label == "Walking"),]
Running = TotalData[(TotalData$Label == "Running"),]

# --- Start making 33 features from 200 obs grouped together---

i=1
nrows=1

dataStationary <- NULL;

while(i+200<dim(Stationary)[1]){
  groupi = Stationary[i:(i+199),-c(4)]
  i = i+200
  
  maxA1 = max(groupi$A1)
  maxA2 = max(groupi$A2)
  maxA3 = max(groupi$A3)
  
  minA1 = min(groupi$A1)
  minA2 = min(groupi$A2)
  minA3 = min(groupi$A3)
  
  difA1 = max(groupi$A1) - min(groupi$A1)
  difA2 = max(groupi$A2) - min(groupi$A2)
  difA3 = max(groupi$A3) - min(groupi$A3)
  
  meanA1 = mean(groupi$A1)
  meanA2 = mean(groupi$A2)
  meanA3 = mean(groupi$A3)
  
  sdA1 = sd(groupi$A1)
  sdA2 = sd(groupi$A2)
  sdA3 = sd(groupi$A3)
  
  # AAD and MAD - Mean (average) absolute deviation from the mean  AND Median Absolute Deviation
  aadA1 = aad(groupi$A1)
  aadA2 = aad(groupi$A2)
  aadA3 = aad(groupi$A3)
  
  madA1 = mad(groupi$A1)
  madA2 = mad(groupi$A2)
  madA3 = mad(groupi$A3)
  
  # The Interquartile Range
  iqrA1 = IQR(groupi$A1)
  iqrA2 = IQR(groupi$A2)
  iqrA3 = IQR(groupi$A3)
  
  rmsA1 = rms(groupi$A1)
  rmsA2 = rms(groupi$A2)
  rmsA3 = rms(groupi$A3)
  
  aucA1 = auc(c(i:(i+199)),groupi$A1)
  aucA2 = auc(c(i:(i+199)),groupi$A2)
  aucA3 = auc(c(i:(i+199)),groupi$A3)
  
  # MEAN CROSSINGS - no idea how to find that? 
  
  corr = cor(groupi, use="complete.obs", method="pearson")  
  corr12 = corr[1,2]
  corr13 = corr[1,3]
  corr23 = corr[2,3]
  
  row = c(maxA1, maxA2, maxA3, minA1, minA2, minA3, difA1, difA2, difA3, meanA1, meanA2, meanA3, sdA1, sdA2, sdA3, aadA1, aadA2, aadA3, madA1, madA2, madA3, iqrA1, iqrA2, iqrA3, rmsA1, rmsA2, rmsA3, aucA1, aucA2, aucA3, corr12, corr13, corr23)
  rbind.data.frame(dataStationary,row)->dataStationary
  nrows = nrows + 1
}

# so now we have less than 200 left to check
groupi = tail(Stationary,200)

maxA1 = max(groupi$A1)
maxA2 = max(groupi$A2)
maxA3 = max(groupi$A3)

minA1 = min(groupi$A1)
minA2 = min(groupi$A2)
minA3 = min(groupi$A3)

difA1 = max(groupi$A1) - min(groupi$A1)
difA2 = max(groupi$A2) - min(groupi$A2)
difA3 = max(groupi$A3) - min(groupi$A3)

meanA1 = mean(groupi$A1)
meanA2 = mean(groupi$A2)
meanA3 = mean(groupi$A3)

sdA1 = sd(groupi$A1)
sdA2 = sd(groupi$A2)
sdA3 = sd(groupi$A3)

# AAD and MAD - Mean (average) absolute deviation from the mean  AND Median Absolute Deviation
aadA1 = aad(groupi$A1)
aadA2 = aad(groupi$A2)
aadA3 = aad(groupi$A3)

madA1 = mad(groupi$A1)
madA2 = mad(groupi$A2)
madA3 = mad(groupi$A3)

# The Interquartile Range
iqrA1 = IQR(groupi$A1)
iqrA2 = IQR(groupi$A2)
iqrA3 = IQR(groupi$A3)

rmsA1 = rms(groupi$A1)
rmsA2 = rms(groupi$A2)
rmsA3 = rms(groupi$A3)

aucA1 = auc(c(i:(i+199)),groupi$A1)
aucA2 = auc(c(i:(i+199)),groupi$A2)
aucA3 = auc(c(i:(i+199)),groupi$A3)

# MEAN CROSSINGS - no idea how to find that? 

corr = cor(groupi, use="complete.obs", method="pearson")  
corr12 = corr[1,2]
corr13 = corr[1,3]
corr23 = corr[2,3]

row = c(maxA1, maxA2, maxA3, minA1, minA2, minA3, difA1, difA2, difA3, meanA1, meanA2, meanA3, sdA1, sdA2, sdA3, aadA1, aadA2, aadA3, madA1, madA2, madA3, iqrA1, iqrA2, iqrA3, rmsA1, rmsA2, rmsA3, aucA1, aucA2, aucA3, corr12, corr13, corr23)
rbind.data.frame(dataStationary,row)->dataStationary

colnames(dataStationary) <- c("maxA1", "maxA2", "maxA3", "minA1", "minA2", "minA3", "difA1", "difA2", "difA3", "meanA1", "meanA2", "meanA3", "sdA1", "sdA2", "sdA3", "aadA1", "aadA2", "aadA3", "madA1", "madA2", "madA3", "iqrA1", "iqrA2", "iqrA3", "rmsA1", "rmsA2", "rmsA3", "aucA1", "aucA2", "aucA3", "corr12", "corr13", "corr23")

# ---- END of stationary and START of walking ---- 


i=1
nrows=1

dataWalking <- NULL;

while(i+200<dim(Walking)[1]){
  groupi = Walking[i:(i+199),-c(4)]
  i = i+200
  
  maxA1 = max(groupi$A1)
  maxA2 = max(groupi$A2)
  maxA3 = max(groupi$A3)
  
  minA1 = min(groupi$A1)
  minA2 = min(groupi$A2)
  minA3 = min(groupi$A3)
  
  difA1 = max(groupi$A1) - min(groupi$A1)
  difA2 = max(groupi$A2) - min(groupi$A2)
  difA3 = max(groupi$A3) - min(groupi$A3)
  
  meanA1 = mean(groupi$A1)
  meanA2 = mean(groupi$A2)
  meanA3 = mean(groupi$A3)
  
  sdA1 = sd(groupi$A1)
  sdA2 = sd(groupi$A2)
  sdA3 = sd(groupi$A3)
  
  # AAD and MAD - Mean (average) absolute deviation from the mean  AND Median Absolute Deviation
  aadA1 = aad(groupi$A1)
  aadA2 = aad(groupi$A2)
  aadA3 = aad(groupi$A3)
  
  madA1 = mad(groupi$A1)
  madA2 = mad(groupi$A2)
  madA3 = mad(groupi$A3)
  
  # The Interquartile Range
  iqrA1 = IQR(groupi$A1)
  iqrA2 = IQR(groupi$A2)
  iqrA3 = IQR(groupi$A3)
  
  rmsA1 = rms(groupi$A1)
  rmsA2 = rms(groupi$A2)
  rmsA3 = rms(groupi$A3)
  
  aucA1 = auc(c(i:(i+199)),groupi$A1)
  aucA2 = auc(c(i:(i+199)),groupi$A2)
  aucA3 = auc(c(i:(i+199)),groupi$A3)
  
  # MEAN CROSSINGS - no idea how to find that? 
  
  corr = cor(groupi, use="complete.obs", method="pearson")  
  corr12 = corr[1,2]
  corr13 = corr[1,3]
  corr23 = corr[2,3]
  
  row = c(maxA1, maxA2, maxA3, minA1, minA2, minA3, difA1, difA2, difA3, meanA1, meanA2, meanA3, sdA1, sdA2, sdA3, aadA1, aadA2, aadA3, madA1, madA2, madA3, iqrA1, iqrA2, iqrA3, rmsA1, rmsA2, rmsA3, aucA1, aucA2, aucA3, corr12, corr13, corr23)
  rbind.data.frame(dataWalking,row)->dataWalking
  nrows = nrows + 1
}

# so now we have less than 200 left to check
groupi = tail(Walking,200)

maxA1 = max(groupi$A1)
maxA2 = max(groupi$A2)
maxA3 = max(groupi$A3)

minA1 = min(groupi$A1)
minA2 = min(groupi$A2)
minA3 = min(groupi$A3)

difA1 = max(groupi$A1) - min(groupi$A1)
difA2 = max(groupi$A2) - min(groupi$A2)
difA3 = max(groupi$A3) - min(groupi$A3)

meanA1 = mean(groupi$A1)
meanA2 = mean(groupi$A2)
meanA3 = mean(groupi$A3)

sdA1 = sd(groupi$A1)
sdA2 = sd(groupi$A2)
sdA3 = sd(groupi$A3)

# AAD and MAD - Mean (average) absolute deviation from the mean  AND Median Absolute Deviation
aadA1 = aad(groupi$A1)
aadA2 = aad(groupi$A2)
aadA3 = aad(groupi$A3)

madA1 = mad(groupi$A1)
madA2 = mad(groupi$A2)
madA3 = mad(groupi$A3)

# The Interquartile Range
iqrA1 = IQR(groupi$A1)
iqrA2 = IQR(groupi$A2)
iqrA3 = IQR(groupi$A3)

rmsA1 = rms(groupi$A1)
rmsA2 = rms(groupi$A2)
rmsA3 = rms(groupi$A3)

aucA1 = auc(c(i:(i+199)),groupi$A1)
aucA2 = auc(c(i:(i+199)),groupi$A2)
aucA3 = auc(c(i:(i+199)),groupi$A3)

# MEAN CROSSINGS - no idea how to find that? 

corr = cor(groupi, use="complete.obs", method="pearson")  
corr12 = corr[1,2]
corr13 = corr[1,3]
corr23 = corr[2,3]

row = c(maxA1, maxA2, maxA3, minA1, minA2, minA3, difA1, difA2, difA3, meanA1, meanA2, meanA3, sdA1, sdA2, sdA3, aadA1, aadA2, aadA3, madA1, madA2, madA3, iqrA1, iqrA2, iqrA3, rmsA1, rmsA2, rmsA3, aucA1, aucA2, aucA3, corr12, corr13, corr23)
rbind.data.frame(dataWalking,row)->dataWalking

colnames(dataWalking) <- c("maxA1", "maxA2", "maxA3", "minA1", "minA2", "minA3", "difA1", "difA2", "difA3", "meanA1", "meanA2", "meanA3", "sdA1", "sdA2", "sdA3", "aadA1", "aadA2", "aadA3", "madA1", "madA2", "madA3", "iqrA1", "iqrA2", "iqrA3", "rmsA1", "rmsA2", "rmsA3", "aucA1", "aucA2", "aucA3", "corr12", "corr13", "corr23")

# ---- END of walking start of running ---- 

i=1
nrows=1

dataRunning <- NULL;

while(i+200<dim(Running)[1]){
  groupi = Running[i:(i+199),-c(4)]
  i = i+200
  
  maxA1 = max(groupi$A1)
  maxA2 = max(groupi$A2)
  maxA3 = max(groupi$A3)
  
  minA1 = min(groupi$A1)
  minA2 = min(groupi$A2)
  minA3 = min(groupi$A3)
  
  difA1 = max(groupi$A1) - min(groupi$A1)
  difA2 = max(groupi$A2) - min(groupi$A2)
  difA3 = max(groupi$A3) - min(groupi$A3)
  
  meanA1 = mean(groupi$A1)
  meanA2 = mean(groupi$A2)
  meanA3 = mean(groupi$A3)
  
  sdA1 = sd(groupi$A1)
  sdA2 = sd(groupi$A2)
  sdA3 = sd(groupi$A3)
  
  # AAD and MAD - Mean (average) absolute deviation from the mean  AND Median Absolute Deviation
  aadA1 = aad(groupi$A1)
  aadA2 = aad(groupi$A2)
  aadA3 = aad(groupi$A3)
  
  madA1 = mad(groupi$A1)
  madA2 = mad(groupi$A2)
  madA3 = mad(groupi$A3)
  
  # The Interquartile Range
  iqrA1 = IQR(groupi$A1)
  iqrA2 = IQR(groupi$A2)
  iqrA3 = IQR(groupi$A3)
  
  rmsA1 = rms(groupi$A1)
  rmsA2 = rms(groupi$A2)
  rmsA3 = rms(groupi$A3)
  
  aucA1 = auc(c(i:(i+199)),groupi$A1)
  aucA2 = auc(c(i:(i+199)),groupi$A2)
  aucA3 = auc(c(i:(i+199)),groupi$A3)
  
  # MEAN CROSSINGS - no idea how to find that? 
  
  corr = cor(groupi, use="complete.obs", method="pearson")  
  corr12 = corr[1,2]
  corr13 = corr[1,3]
  corr23 = corr[2,3]
  
  row = c(maxA1, maxA2, maxA3, minA1, minA2, minA3, difA1, difA2, difA3, meanA1, meanA2, meanA3, sdA1, sdA2, sdA3, aadA1, aadA2, aadA3, madA1, madA2, madA3, iqrA1, iqrA2, iqrA3, rmsA1, rmsA2, rmsA3, aucA1, aucA2, aucA3, corr12, corr13, corr23)
  rbind.data.frame(dataRunning,row)->dataRunning
  nrows = nrows + 1
}

# so now we have less than 200 left to check
groupi = tail(Running,200)

maxA1 = max(groupi$A1)
maxA2 = max(groupi$A2)
maxA3 = max(groupi$A3)

minA1 = min(groupi$A1)
minA2 = min(groupi$A2)
minA3 = min(groupi$A3)

difA1 = max(groupi$A1) - min(groupi$A1)
difA2 = max(groupi$A2) - min(groupi$A2)
difA3 = max(groupi$A3) - min(groupi$A3)

meanA1 = mean(groupi$A1)
meanA2 = mean(groupi$A2)
meanA3 = mean(groupi$A3)

sdA1 = sd(groupi$A1)
sdA2 = sd(groupi$A2)
sdA3 = sd(groupi$A3)

# AAD and MAD - Mean (average) absolute deviation from the mean  AND Median Absolute Deviation
aadA1 = aad(groupi$A1)
aadA2 = aad(groupi$A2)
aadA3 = aad(groupi$A3)

madA1 = mad(groupi$A1)
madA2 = mad(groupi$A2)
madA3 = mad(groupi$A3)

# The Interquartile Range
iqrA1 = IQR(groupi$A1)
iqrA2 = IQR(groupi$A2)
iqrA3 = IQR(groupi$A3)

rmsA1 = rms(groupi$A1)
rmsA2 = rms(groupi$A2)
rmsA3 = rms(groupi$A3)

aucA1 = auc(c(i:(i+199)),groupi$A1)
aucA2 = auc(c(i:(i+199)),groupi$A2)
aucA3 = auc(c(i:(i+199)),groupi$A3)

# MEAN CROSSINGS - no idea how to find that? 

corr = cor(groupi, use="complete.obs", method="pearson")  
corr12 = corr[1,2]
corr13 = corr[1,3]
corr23 = corr[2,3]

row = c(maxA1, maxA2, maxA3, minA1, minA2, minA3, difA1, difA2, difA3, meanA1, meanA2, meanA3, sdA1, sdA2, sdA3, aadA1, aadA2, aadA3, madA1, madA2, madA3, iqrA1, iqrA2, iqrA3, rmsA1, rmsA2, rmsA3, aucA1, aucA2, aucA3, corr12, corr13, corr23)
rbind.data.frame(dataRunning,row)->dataRunning

colnames(dataRunning) <- c("maxA1", "maxA2", "maxA3", "minA1", "minA2", "minA3", "difA1", "difA2", "difA3", "meanA1", "meanA2", "meanA3", "sdA1", "sdA2", "sdA3", "aadA1", "aadA2", "aadA3", "madA1", "madA2", "madA3", "iqrA1", "iqrA2", "iqrA3", "rmsA1", "rmsA2", "rmsA3", "aucA1", "aucA2", "aucA3", "corr12", "corr13", "corr23")

# --- End of making data -- 

dataRunning <- data.frame(dataRunning, label = rep("R",dim(dataRunning)[1]))
dataWalking <- data.frame(dataWalking, label = rep("W",dim(dataWalking)[1]))
dataStationary <- data.frame(dataStationary, label = rep("R",dim(dataStationary)[1]))

enggData =  rbind.data.frame(dataRunning,dataWalking,dataStationary)

inTraining <- createDataPartition(enggData$label, p = .8, list = FALSE)
training <- enggData[ inTraining,]
testing  <- enggData[-inTraining,]

train.predictors = training[,-c(34)]
train.response = training[,34]

test.predictors = testing[,-c(34)]
test.response = testing[,34]

k = 5
# I use K-fold cross validation - "cv"
train_control <- trainControl(method='cv', number=k, returnResamp='none', classProbs = TRUE)

# I train the naive bayes model
fit_nb <- train(train.predictors, train.response, method = "nb", trControl=train_control, metric = "ROC")

pred_nb = predict(fit_nb, test.predictors)
error_nb = mean(test.response != pred_nb)*100

cat("Error rate for normal Naive Bayes: ", error_nb)