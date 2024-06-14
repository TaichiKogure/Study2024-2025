import numpy as np
import tensorflow as tf

print(tf.__version__)

gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    logical_gpus = tf.config.experimental.list_logical_devices('GPU')
    print("Physical GPUs: {}, Logical GPUs: {}".format(len(gpus), len(logical_gpus)))
else:
    print("CPU only")

x = np.arange(-1, 1, 0.0001)
y = 0.8 * x + 0.2

model = tf.keras.Sequential([tf.keras.layers.Dense(1, activation=None)])
model.compile("sgd", "mse")
model.build(input_shape=(0,1))
model.summary()
model.fit(x, y, epochs=5)

print("ground truth: 0.8, 0.2")
print("estimated: ", model.variables[0][0,0].numpy(), model.variables[1][0].numpy())
#%%
#Update test
# TensorFlow and tf.keras
import tensorflow as tf
from tensorflow import keras

# Helper libraries
import numpy as np
import matplotlib.pyplot as plt

fashion_mnist = keras.datasets.fashion_mnist

(train_images, train_labels), (test_images, test_labels) = fashion_mnist.load_data()
class_names = ['T-shirt/top', 'Trouser', 'Pullover', 'Dress', 'Coat',
               'Sandal', 'Shirt', 'Sneaker', 'Bag', 'Ankle boot']
train_images = train_images / 255.0

test_images = test_images / 255.0

model = keras.Sequential([
    keras.layers.Flatten(input_shape=(28, 28)),
    keras.layers.Dense(128, activation='relu'),
    keras.layers.Dense(10, activation='softmax')
])
model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])
model.fit(train_images, train_labels, epochs=5)
predictions = model.predict(test_images)
def plot_image(i, predictions_array, true_label, img):
    predictions_array, true_label, img = predictions_array[i], true_label[i], img[i]
    plt.grid(False)
    plt.xticks([])
    plt.yticks([])

    plt.imshow(img, cmap=plt.cm.binary)

    predicted_label = np.argmax(predictions_array)
    if predicted_label == true_label:
        color = 'blue'
    else:
        color = 'red'

    plt.xlabel("{} {:2.0f}% ({})".format(class_names[predicted_label],
                                    100*np.max(predictions_array),
                                    class_names[true_label]),
                                    color=color)

def plot_value_array(i, predictions_array, true_label):
    predictions_array, true_label = predictions_array[i], true_label[i]
    plt.grid(False)
    plt.xticks([])
    plt.yticks([])

    thisplot = plt.bar(range(10), predictions_array, color="#777777")
    plt.ylim([0, 1])
    predicted_label = np.argmax(predictions_array)

    thisplot[predicted_label].set_color('red')
    thisplot[true_label].set_color('blue')


# X個のテスト画像、予測されたラベル、正解ラベルを表示します。
# 正しい予測は青で、間違った予測は赤で表示しています。
num_rows = 5
num_cols = 3
num_images = num_rows*num_cols
plt.figure(figsize=(2*2*num_cols, 2*num_rows))
for i in range(num_images):
  plt.subplot(num_rows, 2*num_cols, 2*i+1)
  plot_image(i, predictions, test_labels, test_images)
  plt.subplot(num_rows, 2*num_cols, 2*i+2)
  plot_value_array(i, predictions, test_labels)
plt.show()

#%%
import numpy as np
import random
import tensorflow as tf

def create_feature_sets_and_labels(test_size = 0.3):

    # known patterns (5 features) output of [1] of positions [0,4]==1
    features = []
    features.append([[0, 0, 0, 0, 0], [0,1]])
    features.append([[0, 0, 0, 0, 1], [0,1]])
    features.append([[0, 0, 0, 1, 1], [0,1]])
    features.append([[0, 0, 1, 1, 1], [0,1]])
    features.append([[0, 1, 1, 1, 1], [0,1]])
    features.append([[1, 1, 1, 1, 0], [0,1]])
    features.append([[1, 1, 1, 0, 0], [0,1]])
    features.append([[1, 1, 0, 0, 0], [0,1]])
    features.append([[1, 0, 0, 0, 0], [0,1]])
    features.append([[1, 0, 0, 1, 0], [0,1]])
    features.append([[1, 0, 1, 1, 0], [0,1]])
    features.append([[1, 1, 0, 1, 0], [0,1]])
    features.append([[0, 1, 0, 1, 1], [0,1]])
    features.append([[0, 0, 1, 0, 1], [0,1]])
    features.append([[1, 0, 1, 1, 1], [1,0]])
    features.append([[1, 1, 0, 1, 1], [1,0]])
    features.append([[1, 0, 1, 0, 1], [1,0]])
    features.append([[1, 0, 0, 0, 1], [1,0]])
    features.append([[1, 1, 0, 0, 1], [1,0]])
    features.append([[1, 1, 1, 0, 1], [1,0]])
    features.append([[1, 1, 1, 1, 1], [1,0]])
    features.append([[1, 0, 0, 1, 1], [1,0]])

    # shuffle out features and turn into np.array
    random.shuffle(features)
    features = np.array(features)

    # split a portion of the features into tests
    testing_size = int(test_size*len(features))

    # create train and test lists
    train_x = list(features[:,0][:-testing_size])
    train_y = list(features[:,1][:-testing_size])
    test_x = list(features[:,0][-testing_size:])
    test_y = list(features[:,1][-testing_size:])

    return train_x, train_y, test_x, test_y


train_x, train_y, test_x, test_y = create_feature_sets_and_labels()

# hidden layers and their nodes
n_nodes_hl1 = 20
n_nodes_hl2 = 20

# classes in our output
n_classes = 2
# iterations and batch-size to build out model
hm_epochs = 1000
batch_size = 4

x = tf.placeholder('float')
y = tf.placeholder('float')

# random weights and bias for our layers
hidden_1_layer = {'f_fum': n_nodes_hl1,
                  'weight': tf.Variable(tf.random_normal([len(train_x[0]), n_nodes_hl1])),
                  'bias': tf.Variable(tf.random_normal([n_nodes_hl1]))}

hidden_2_layer = {'f_fum': n_nodes_hl2,
                  'weight': tf.Variable(tf.random_normal([n_nodes_hl1, n_nodes_hl2])),
                  'bias': tf.Variable(tf.random_normal([n_nodes_hl2]))}

output_layer = {'f_fum': None,
                'weight': tf.Variable(tf.random_normal([n_nodes_hl2, n_classes])),
                'bias': tf.Variable(tf.random_normal([n_classes])), }

# our predictive model's definition
def neural_network_model(data):

    # hidden layer 1: (data * W) + b
    l1 = tf.add(tf.matmul(data,hidden_1_layer['weight']), hidden_1_layer['bias'])
    l1 = tf.sigmoid(l1)

    # hidden layer 2: (hidden_layer_1 * W) + b
    l2 = tf.add(tf.matmul(l1,hidden_2_layer['weight']), hidden_2_layer['bias'])
    l2 = tf.sigmoid(l2)

    # output: (hidden_layer_2 * W) + b
    output = tf.matmul(l2,output_layer['weight']) + output_layer['bias']

    return output

# hidden layer 1: (data * W) + b

l1 = tf.add(tf.matmul(data, hidden_1_layer['weight']), hidden_1_layer['bias'])
l1 = tf.sigmoid(l1)

# training our model
def train_neural_network(x):
    # use the model definition
    prediction = neural_network_model(x)

    # formula for cost (error)
    cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(prediction, y))
    # optimize for cost using GradientDescent
    optimizer = tf.train.GradientDescentOptimizer(1).minimize(cost)

    # Tensorflow session
    with tf.Session() as sess:
        # initialize our variables
        sess.run(tf.global_variables_initializer())

        # loop through specified number of iterations
        for epoch in range(hm_epochs):
            epoch_loss = 0
            i = 0
            # handle batch sized chunks of training data
            while i  len(train_x):
                start = i
                end = i + batch_size
                batch_x = np.array(train_x[start:end])
                batch_y = np.array(train_y[start:end])

                _, c = sess.run([optimizer, cost], feed_dict={x: batch_x, y: batch_y})
                epoch_loss += c
                i += batch_size
                last_cost = c

            # print cost updates along the way
            if (epoch % (hm_epochs / 5)) == 0:
                print('Epoch', epoch, 'completed out of', hm_epochs, 'cost:', last_cost)

        # print accuracy of our model
        correct = tf.equal(tf.argmax(prediction, 1), tf.argmax(y, 1))
        accuracy = tf.reduce_mean(tf.cast(correct, 'float'))
        print('Accuracy:', accuracy.eval({x: test_x, y: test_y}))

        # print predictions using our model
        for i, t in enumerate(test_x):
            print('prediction for:', test_x[i])
            output = prediction.eval(feed_dict={x: [test_x[i]]})
            # normalize the prediction values
            print(tf.sigmoid(output[0][0]).eval(), tf.sigmoid(output[0][1]).eval())


train_neural_network(x)


#%%

import tensorflow as tf

mnist = tf.keras.datasets.mnist

(x_train, y_train), (x_test, y_test) = mnist.load_data() #データの読み込み
x_train, x_test = x_train / 255.0, x_test / 255.0 #データの正規化


# モデルの構築
# Sequential API
model = tf.keras.models.Sequential([
  tf.keras.layers.Flatten(input_shape=(28, 28)),
  tf.keras.layers.Dense(128, activation='relu'),
  tf.keras.layers.Dropout(0.2),
  tf.keras.layers.Dense(10, activation='softmax')
])


# optimizer, loss, metricsの設定（学習設定）
model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

# 学習
model.fit(x_train, y_train, epochs=5)

# 評価
test_loss, test_acc = model.evaluate(x_test,  y_test, verbose=2)

print('\nTest accuracy:', test_acc)

#%%
#Pythonの pandas ライブラリを使用することで、複数のエクセルデータを簡単にマージすることができます。
# 以下に具体的な手順を示します。 まず、pandasをインポートし、エクセルデータをDataFrameとして読み込みます。
import pandas as pd

df1 = pd.read_excel('file1.xlsx')
df2 = pd.read_excel('file2.xlsx')
#次に、pandasの merge ファンクションを用いて2つのデータフレームをマージします。
# マージキーとなる列を指定するために on 引数を使用します
merged_df = pd.merge(df1, df2, on='merge_key_column')
# これで merged_df には2つのエクセルデータがマージされた結果が格納されます。
# なお、 merge_key_column は実際のエクセルデータの列名に書き換えて下さい。

#%%
