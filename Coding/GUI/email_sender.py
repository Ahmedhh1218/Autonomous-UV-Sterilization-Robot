import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText

def send_email():
    # Email configuration
    smtp_server = 'smtp.office365.com'  # Outlook SMTP server
    smtp_port = 587  # Use port 587 for TLS
    sender_email = 'SteriBot1@outlook.com'
    sender_password = 'SteriBot@startup1'
    recipient_email = 'ahmedhassabbou2001@gmail.com'

    # Create the email content
    message = MIMEMultipart()
    message['From'] = sender_email
    message['To'] = recipient_email
    message['Subject'] = 'Test Email'

    # Email body
    body = 'This is a test email sent from Python!'
    message.attach(MIMEText(body, 'plain'))

    try:
        # Connect to the server
        server = smtplib.SMTP(smtp_server, smtp_port)
        server.starttls()  # Secure the connection
        server.login(sender_email, sender_password)

        # Send the email
        server.send_message(message)
        print('Email sent successfully!')

    except Exception as e:
        print(f'Error: {e}')

    finally:
        server.quit()

if __name__ == '__main__':
    send_email()
