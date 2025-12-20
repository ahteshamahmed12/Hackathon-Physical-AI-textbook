import React, { useState } from 'react';
import Layout from '@theme/Layout';

function SignUp() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  
  // New states for hardware questions
  const [hasPC, setHasPC] = useState(false);
  const [hasRobot, setHasRobot] = useState(false);

  const handleSignUp = (e: React.FormEvent) => {
    e.preventDefault();
    if (password !== confirmPassword) {
      alert("Passwords do not match!");
      return;
    }

    // Combine all data for submission
    const signupData = {
      email,
      password,
      hardware: {
        pc_available: hasPC,
        robot_available: hasRobot
      }
    };

    console.log('Form Submitted:', signupData);
    alert('Account details sent to console!');
  };

  return (
    <Layout title="Sign Up" description="Create a new account">
      <main
        style={{
          display: 'flex',
          flexDirection: 'column',
          justifyContent: 'center',
          alignItems: 'center',
          minHeight: '80vh', // Increased height to fit new fields
          padding: '20px',
        }}>
        <h1>Sign Up</h1>
        <form onSubmit={handleSignUp} style={{ display: 'flex', flexDirection: 'column', gap: '15px', width: '320px', padding: '20px', border: '1px solid #ccc', borderRadius: '8px', boxShadow: '0 4px 8px rgba(0,0,0,0.1)' }}>
          
          {/* Email Field */}
          <div style={{ display: 'flex', flexDirection: 'column' }}>
            <label htmlFor="email" style={{ marginBottom: '5px', fontWeight: 'bold' }}>Email:</label>
            <input
              type="email"
              id="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              style={{ padding: '10px', borderRadius: '4px', border: '1px solid #ddd', fontSize: '16px' }}
              required
            />
          </div>

          {/* Password Fields */}
          <div style={{ display: 'flex', flexDirection: 'column' }}>
            <label htmlFor="password" style={{ marginBottom: '5px', fontWeight: 'bold' }}>Password:</label>
            <input
              type="password"
              id="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              style={{ padding: '10px', borderRadius: '4px', border: '1px solid #ddd', fontSize: '16px' }}
              required
            />
          </div>

          <div style={{ display: 'flex', flexDirection: 'column' }}>
            <label htmlFor="confirmPassword" style={{ marginBottom: '5px', fontWeight: 'bold' }}>Confirm Password:</label>
            <input
              type="password"
              id="confirmPassword"
              value={confirmPassword}
              onChange={(e) => setConfirmPassword(e.target.value)}
              style={{ padding: '10px', borderRadius: '4px', border: '1px solid #ddd', fontSize: '16px' }}
              required
            />
          </div>

          {/* New Hardware Section */}
          <div style={{ marginTop: '10px', padding: '10px', backgroundColor: '#f9f9f9', borderRadius: '5px' }}>
            <p style={{ fontWeight: 'bold', marginBottom: '10px', fontSize: '0.9rem' }}>Hardware Status:</p>
            
            <div style={{ display: 'flex', alignItems: 'center', marginBottom: '8px' }}>
              <input
                type="checkbox"
                id="pcCheckbox"
                checked={hasPC}
                onChange={(e) => setHasPC(e.target.checked)}
                style={{ marginRight: '10px', width: '18px', height: '18px' }}
              />
              <label htmlFor="pcCheckbox" style={{ fontSize: '14px' }}>I have a PC</label>
            </div>

            <div style={{ display: 'flex', alignItems: 'center' }}>
              <input
                type="checkbox"
                id="robotCheckbox"
                checked={hasRobot}
                onChange={(e) => setHasRobot(e.target.checked)}
                style={{ marginRight: '10px', width: '18px', height: '18px' }}
              />
              <label htmlFor="robotCheckbox" style={{ fontSize: '14px' }}>I have a physical robot</label>
            </div>
          </div>

          <button type="submit" className="button button--primary" style={{ padding: '10px', fontSize: '18px', cursor: 'pointer', marginTop: '10px' }}>
            Sign Up
          </button>
        </form>
      </main>
    </Layout>
  );
}

export default SignUp;