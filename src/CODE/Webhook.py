from flask import Flask, request, jsonify
import requests

app = Flask(__name__)

WEBHOOK_URL = "https://discord.com/api/webhooks/1393706088897187841/3LtR4s213JVIYxaWjGvqO0pkpDg_-FjUib2q7KsIePC1etmR2jgYparEMTy9i-FvezOH"

@app.route('/send', methods=['POST'])
def send_command():
    data = request.get_json()
    username = data.get('username')
    command = data.get('command')

    if not username or not command:
        return jsonify({'error': 'Missing username or command'}), 400

    payload = {
        "username": username,
        "content": command,
        "avatar_url": f"https://i.pravatar.cc/150?u={username}"
    }

    try:
        r = requests.post(WEBHOOK_URL, json=payload)
        r.raise_for_status()
        return jsonify({'message': 'Sent to Discord!'}), 200
    except Exception as e:
        print(f"Error sending message: {e}")
        return jsonify({'error': 'Failed to send message'}), 500

if __name__ == '__main__':
    app.run(debug=True, port=3000)