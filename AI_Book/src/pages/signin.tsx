import React, { useState } from 'react';
import Layout from '@theme/Layout';

function SignIn() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');

  const handleSignIn = async (e: React.FormEvent<HTMLFormElement>) => {
    e.preventDefault();

    const formData = new URLSearchParams();
    formData.append('username', email);
    formData.append('password', password);

    try {
      const response = await fetch('http://localhost:8000/signin', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: formData,
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Sign in failed');
      }

      const data = await response.json();
      localStorage.setItem('access_token', data.access_token);
      window.location.href = '/';
    } catch (error) {
      alert(error.message);
    }
  };

  return (
    <Layout title="Sign In" description="Sign In to your account">
      <main
        style={{
          display: 'flex',
          flexDirection: 'column',
          justifyContent: 'center',
          alignItems: 'center',
          height: '70vh',
          padding: '20px',
        }}>
        <h1>Sign In</h1>
        <form 
          onSubmit={handleSignIn} 
          style={{ 
            display: 'flex', 
            flexDirection: 'column', 
            gap: '15px', 
            width: '300px', 
            padding: '20px', 
            border: '1px solid #ccc', 
            borderRadius: '8px', 
            boxShadow: '0 4px 8px rgba(0,0,0,0.1)' 
          }}
        >
          <div style={{ display: 'flex', flexDirection: 'column' }}>
            <label htmlFor="email" style={{ marginBottom: '5px', fontWeight: 'bold' }}>Email:</label>
            <input
              type="email"
              id="email"
              value={email}
              // Added React.ChangeEvent type for the input
              onChange={(e: React.ChangeEvent<HTMLInputElement>) => setEmail(e.target.value)}
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
              onChange={(e: React.ChangeEvent<HTMLInputElement>) => setPassword(e.target.value)}
              style={{ padding: '10px', borderRadius: '4px', border: '1px solid #ddd', fontSize: '16px' }}
              required
            />
          </div>
          {/* Fixed the class name from button--primary-1 to button--primary */}
          <button type="submit" className="button button--primary" style={{ padding: '10px', fontSize: '18px', cursor: 'pointer', marginTop: '10px' }}>
            Sign In
          </button>
        </form>
      </main>
    </Layout>
  );
}

export default SignIn;