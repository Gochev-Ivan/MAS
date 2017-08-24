import csv
import matplotlib.pyplot as plt
import numpy as np
from sklearn import datasets, linear_model

X_train = []
Y_train = []
X_test = []
Y_test = []
for k in range(1,35):
    f = 'GD_'+str(k)+'.csv'
    with open(f,'rb') as csvfile:
        rdr = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in rdr:
            for i in xrange(len(row)):
                row[i] = float(row[i])
            X_train.append(row[:8])
            Y_train.append(row[8:12])
                
for k in range(35,43):
    f = 'GD_'+str(k)+'.csv'
    with open(f,'rb') as csvfile:
        rdr = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in rdr:
            for i in xrange(len(row)):
                row[i] = float(row[i])
            print row[:8]
#            print row[8:12]
            X_test.append(row[:8])
            Y_test.append(row[8:12])

#print X_test
#print len(X_train), len(Y_train)
print len(X_test), len(Y_test)
X_test = np.array(X_test)
Y_test = np.array(Y_test)

X_train = np.array(X_train)
Y_train = np.array(Y_train)
# Create linear regression object
regr = linear_model.LinearRegression()

# Train the model using the training sets
regr.fit(X_train, Y_train)

# The coefficients
print('Coefficients: \n', regr.coef_)
# The mean squared error
print("Mean squared error: %.2f"
      % np.mean((regr.predict(X_test) - Y_test) ** 2))
# Explained variance score: 1 is perfect prediction
print('Variance score: %.2f' % regr.score(X_test, Y_test))

Y_predicted = regr.predict(X_train)
print Y_predicted[:,0]
print Y_test[:,0]

#print Y_train-Y_predict
# %%
# Plot outputs
plt.scatter(X_test, Y_test,  color='black')
plt.plot(X_test, regr.predict(X_test), color='blue',
         linewidth=3)

plt.xticks(())
plt.yticks(())

plt.show()
# %%
Y = []
X = []            
u = 0
j = 2
while u < len(vlezni):
    X.append(vlezni[u])
    X.append(vlezni[u+1])
    while j < u:
        Y.append(vlezni[j])
        j= j + 1
    j = j + 3
    u = u + 12
#do ovde e sve gucci

print X  
print Y 

# Create linear regression object
regr = linear_model.LinearRegression()

# Train the model using the training sets
regr.fit(X, Y)

# The coefficients
print('Coefficients: \n', regr.coef_)
# The mean squared error
print("Mean squared error: %.2f"
      % np.mean((regr.predict(X) - Y) ** 2))
# Explained variance score: 1 is perfect prediction
print('Variance score: %.2f' % regr.score(X, Y))

# Plot outputs
plt.scatter(X, Y,  color='black')
plt.plot(X, regr.predict(X), color='blue',
         linewidth=3)

plt.xticks(())
plt.yticks(())

plt.show()
                


#for i in vlezni:
#    print i 