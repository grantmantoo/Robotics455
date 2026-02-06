from flask import Flask, request, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

@app.route('/check', methods=['GET'])
def check():
    return "Hello"

@app.route('/receive', methods=['POST'])
def receive():
    
    data = request.json  # Get JSON data from the request
    message = data.get('message')  # Extract the 'message' value
    print(message)  # Print only the message string
    return jsonify({"response": f"Received: {data.get('message', 'No message')}"})

if __name__ == '__main__':
    app.run(host='127.0.0.1', port=5001, debug=True)
