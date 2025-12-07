import React, { useState } from 'react';
import Layout from '@theme/Layout';

function SignUp() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');

  const handleSignUp = (e: React.FormEvent) => {
    e.preventDefault();
    if (password !== confirmPassword) {
      alert("Passwords do not match!");
      return;
    }
    console.log('Sign Up Attempt:', { email, password });
    // Placeholder for API call
    alert('Sign Up functionality is not yet implemented. Check console for details.');
  };

  return (
    <Layout title="Sign Up" description="Create a new account">
      <main
        style={{
          display: 'flex',
          flexDirection: 'column',
          justifyContent: 'center',
          alignItems: 'center',
          height: '70vh',
          padding: '20px',
        }}>
        <h1>Sign Up</h1>
        <form onSubmit={handleSignUp} style={{ display: 'flex', flexDirection: 'column', gap: '15px', width: '300px', padding: '20px', border: '1px solid #ccc', borderRadius: '8px', boxShadow: '0 4px 8px rgba(0,0,0,0.1)' }}>
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
          <button type="submit" className="button button--primary" style={{ padding: '10px', fontSize: '18px', cursor: 'pointer', marginTop: '10px' }}>
            Sign Up
          </button>
        </form>
      </main>
    </Layout>
  );
}

export default SignUp;
