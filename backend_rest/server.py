from flask import Flask, jsonify, request
import openai
import numpy as np
import os
from flask_cors import CORS, cross_origin

app = Flask(__name__)

cors = CORS(app, resources={
    r"/api/openai_service": {"origins": "*"},
})

app.config['CORS_HEADERS'] = 'Content-Type'


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
        model="gpt-4",
        messages=[{"role": "system", "content": 'You are an expert in summarizing document and generating a short word list for someone learning foreign languages'},
                      {"role": "user", "content": str(data)}]
    )
    response = {'output': str(gpt_response["choices"][0]["message"]["content"])}
    return jsonify(response)


# @cross_origin(origin='*',headers=['Content-Type','Authorization'])
# @app.route('/api/dalle_service', methods=['POST'])
# def post_dalle():
#     openai.api_key = os.getenv("OPENAI_API_KEY")
#     PROMPT = request.get_json()["input"]

    
#     print("request received")
    
#     print(PROMPT)

#     response = openai.Image.create(
#         prompt=PROMPT,
#         n=1,
#         size="256x256",
#     )

#     print(response["data"][0]["url"])
#     return response

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)