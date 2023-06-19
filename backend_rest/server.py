from flask import Flask, jsonify, request
import openai
import numpy as np
import os
from flask_cors import CORS, cross_origin
import requests

app = Flask(__name__)

cors = CORS(app, resources={
    r"/api/openai_service": {"origins": "*"},
    r"/api/falcon_service": {"origins": "*"},
})

app.config['CORS_HEADERS'] = 'Content-Type'


def query(payload):
    API_URL = "https://api-inference.huggingface.co/models/tiiuae/falcon-7b-instruct"
    headers = {"Authorization": "Bearer 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"}
    response = requests.post(API_URL, headers=headers, json=payload)
    return response.json()


@app.route('/')
def index():
    return 'invalid call'


@cross_origin(origin='*',headers=['Content-Type','Authorization'])
# Define a route that accepts POST requests
@app.route('/api/openai_service', methods=['POST'])
def post_openai():
    openai.api_key = os.getenv("OPENAI_API_KEY")
    # Access request data
    data = request.get_json()["user_input"]
    
    print("request received")

    print(data)

    gpt_response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[{"role": "system", "content": 'You are an expert in summarizing document and generating a short word list for someone learning foreign languages'},
                      {"role": "user", "content": "generate a short word list containing 10 words from the following words, do not say anything else: " + str(data)}]
    )
    response = {'output': str(gpt_response["choices"][0]["message"]["content"])}
    return jsonify(response)

@cross_origin(origin='*',headers=['Content-Type','Authorization'])
# Define a route that accepts POST requests
@app.route('/api/falcon_service', methods=['POST'])
def post_falcon():
    openai.api_key = os.getenv("OPENAI_API_KEY")
    # Access request data
    data = request.get_json()["user_input"]
    
    print("request received")

    print(data)
    response = query({
        "inputs": "give me a list of 10 words without repetition that has strong correlation with the following words: cup, laptop, monitor, cell phone",
    })
    return jsonify(response)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)